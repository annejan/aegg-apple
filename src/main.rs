//! Bad Apple!! on the CyberAegg badge.
//!
//! Plays `BADAPPLE.VID` on the e-paper panel while `BADAPPLE.SND` comes out of
//! the piezo, both streamed from the badge's FAT12 volume on the external QSPI
//! flash.
//!
//! The two are not driven from a shared timer. Audio runs freely out of its
//! DMA buffers and acts as the master clock; the video loop asks how many
//! samples have played and shows whichever frame that lands on. When the panel
//! cannot keep up -- and against a 250 ms frame interval it frequently cannot
//! -- frames are skipped rather than queued, so the picture stays with the
//! music instead of sliding steadily behind it.

#![no_std]
#![no_main]

mod adpcm;
mod asset;
mod audio;
mod board;
mod epd;
mod fat12;
mod video;

mod flash;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use {defmt_rtt as _, panic_probe as _};

use adpcm::SoundHeader;
use asset::AssetRead;
use video::{FRAME_BYTES, VideoHeader};

/// Largest coded frame the player will accept.
///
/// Thresholded video cannot approach this -- the worst real frame is a few
/// hundred bytes -- so it exists only to bound the read buffer if the file is
/// corrupt.
const MAX_FRAME_BYTES: usize = 8 * 1024;

/// LUT timing scale used for playback refreshes.
///
/// 30 is the fastest the waveform engine permits, but a lighter waveform also
/// drives the pixels less far -- at the fast end the image can be too faint to
/// read. Start at the OEM timing, which is definitely visible, and tune down
/// once the picture is confirmed on the panel.
const LUT_SPEED: u8 = 100;

/// Backstop for a single panel refresh. Longer than the slowest full
/// waveform (~6 s) so it only fires on a genuinely stuck controller.
const REFRESH_TIMEOUT: Duration = Duration::from_secs(15);

/// A file on the badge's FAT12 volume, read a piece at a time.
#[derive(Clone, Copy)]
struct FatFile(fat12::FileRef);

impl AssetRead for FatFile {
    async fn read_at(&mut self, offset: u32, buf: &mut [u8]) -> usize {
        // A failed read is reported as a short read, which both players treat
        // as end of stream -- better than feeding them stale buffer contents.
        fat12::read_at(&self.0, offset, buf).await.unwrap_or(0)
    }
}

#[embassy_executor::task]
async fn audio_task(
    pwm: embassy_nrf::Peri<'static, peripherals::PWM0>,
    pin: embassy_nrf::Peri<'static, peripherals::P0_13>,
    file: FatFile,
    header: SoundHeader,
) {
    audio::play(pwm, pin, file, header).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // LEDs are active low: blue on while loading, red blinks on failure.
    let mut led_blue = Output::new(board!(p, led_blue), Level::Low, OutputDrive::Standard);
    let mut led_red = Output::new(board!(p, led_red), Level::High, OutputDrive::Standard);
    let mut led_green = Output::new(board!(p, led_green), Level::High, OutputDrive::Standard);

    let fire = Input::new(board!(p, joy_fire), Pull::Up);

    flash::init(
        p.QSPI,
        board!(p, flash_sck),
        board!(p, flash_csn),
        board!(p, flash_io0),
        board!(p, flash_io1),
        board!(p, flash_io2),
        board!(p, flash_io3),
    )
    .await;

    let Some((video_file, video_header)) = open_video().await else {
        fail(&mut led_red).await
    };
    let Some((sound_file, sound_header)) = open_sound().await else {
        fail(&mut led_red).await
    };

    defmt::info!(
        "{} frames @ {} fps, {} samples @ {} Hz",
        video_header.frame_count,
        video_header.fps,
        sound_header.sample_count,
        sound_header.sample_rate
    );

    let mut panel = epd::Panel::init(
        board!(p, epd_spi),
        board!(p, epd_sck).into(),
        board!(p, epd_mosi).into(),
        board!(p, epd_csn).into(),
        board!(p, epd_dc).into(),
        board!(p, epd_reset).into(),
        board!(p, epd_busy).into(),
    )
    .await;

    // Start from clean white: a flashing full refresh clears whatever ghost
    // the previous firmware left behind.
    panel.clear_white().await;
    led_blue.set_high();

    spawner.must_spawn(audio_task(
        board!(p, buzzer_pwm),
        board!(p, buzzer),
        sound_file,
        sound_header,
    ));

    let stop = play(
        &mut panel,
        video_file,
        video_header,
        sound_header.sample_rate,
        &fire,
        &mut led_green,
        &mut led_blue,
    )
    .await;

    // There is no debug probe on this badge, so the reason playback ended is
    // reported by blinking it out on the red LED. The last frame is left on
    // the panel deliberately -- clearing it would throw away evidence.
    defmt::info!("stopped: {}", stop as u8);
    loop {
        for _ in 0..stop as u8 {
            led_red.set_low();
            Timer::after(Duration::from_millis(200)).await;
            led_red.set_high();
            Timer::after(Duration::from_millis(200)).await;
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

/// Why playback ended. Blinked out on the red LED, so the values are the
/// blink counts.
#[derive(Clone, Copy, PartialEq)]
enum Stop {
    /// Ran to the end of the video. The happy path.
    Complete = 1,
    /// Fire was held.
    Fire = 2,
    /// The audio stream ended first.
    AudioEnded = 3,
    /// The offset table or frame data could not be read.
    ReadFailed = 4,
    /// A frame's run coding did not describe a whole frame.
    DecodeFailed = 5,
}

/// Drive the video stream against the audio clock until it ends, the stream
/// runs out, or Fire is pressed.
async fn play(
    panel: &mut epd::Panel,
    mut file: FatFile,
    header: VideoHeader,
    sample_rate: u32,
    fire: &Input<'_>,
    led_green: &mut Output<'static>,
    led_blue: &mut Output<'static>,
) -> Stop {
    let mut plane = [0u8; FRAME_BYTES];
    let mut encoded = [0u8; MAX_FRAME_BYTES];

    let mut shown: Option<u32> = None;
    let mut skipped: u32 = 0;
    let started = Instant::now();
    let mut warned_silent = false;
    let mut stalled: u32 = 0;

    loop {
        if fire.is_low() {
            return Stop::Fire;
        }
        if audio::finished() {
            return Stop::AudioEnded;
        }

        // Whichever frame the music has reached.
        //
        // If audio never starts -- a dead PWM path, a missing file -- the
        // sample counter stays at zero and would pin the video on frame 0
        // forever. Video is worth watching without sound, so fall back to the
        // wall clock once it is clear no samples are coming.
        let played = audio::samples_played() as u64;
        let target = if played > 0 {
            (played * header.fps as u64 / sample_rate.max(1) as u64) as u32
        } else if started.elapsed() > Duration::from_millis(1500) {
            if !warned_silent {
                defmt::warn!("no audio after 1.5 s; running video on the wall clock");
                warned_silent = true;
            }
            let ms = started.elapsed().as_millis();
            (ms * header.fps as u64 / 1000) as u32
        } else {
            0
        };

        if target >= header.frame_count {
            return Stop::Complete;
        }

        if shown == Some(target) {
            // Ahead of the music: wait instead of redrawing the same frame,
            // which would cost a refresh and gain nothing.
            Timer::after(Duration::from_millis(20)).await;
            continue;
        }

        if let Some(prev) = shown {
            skipped += target.saturating_sub(prev).saturating_sub(1);
        }
        shown = Some(target);

        let Some(len) = read_frame(&mut file, &header, target, &mut encoded).await else {
            defmt::warn!("frame {} unreadable", target);
            return Stop::ReadFailed;
        };

        if video::decode_frame(&encoded[..len], &mut plane).is_none() {
            defmt::warn!("frame {} did not decode", target);
            return Stop::DecodeFailed;
        }

        // Green toggles before the refresh and blue after it, so the pair
        // distinguishes "never reached the panel" from "the panel refresh
        // never returned" without a debug probe.
        led_green.toggle();
        // Outer backstop. The driver's own BUSY waits are bounded, but this
        // guarantees the loop keeps running whatever the panel does -- a
        // stalled refresh costs one frame, not the whole video.
        if with_timeout(REFRESH_TIMEOUT, panel.show(&plane, LUT_SPEED))
            .await
            .is_err()
        {
            stalled += 1;
            defmt::warn!("refresh timed out on frame {} ({} so far)", target, stalled);
        }
        led_blue.toggle();

        defmt::info!("frame {} ({} skipped)", target, skipped);
    }
}

/// Read one coded frame via the offset table, returning its length.
async fn read_frame(
    file: &mut FatFile,
    header: &VideoHeader,
    frame: u32,
    out: &mut [u8; MAX_FRAME_BYTES],
) -> Option<usize> {
    // The table carries frame_count + 1 entries, so both bounds of any valid
    // frame are present and the length is simply their difference.
    let mut bounds = [0u8; 8];
    if file.read_at(header.offset_entry(frame), &mut bounds).await < 8 {
        return None;
    }

    let start = u32::from_le_bytes(bounds[0..4].try_into().ok()?);
    let end = u32::from_le_bytes(bounds[4..8].try_into().ok()?);

    let len = end.checked_sub(start)? as usize;
    if len == 0 || len > MAX_FRAME_BYTES {
        return None;
    }

    (file.read_at(start, &mut out[..len]).await == len).then_some(len)
}

async fn open_video() -> Option<(FatFile, VideoHeader)> {
    let mut file = FatFile(fat12::find_file(&fat12::to_8_3("BADAPPLE.VID")?).await.ok()?);
    let mut head = [0u8; video::HEADER_LEN];
    if file.read_at(0, &mut head).await < head.len() {
        return None;
    }
    Some((file, VideoHeader::parse(&head)?))
}

async fn open_sound() -> Option<(FatFile, SoundHeader)> {
    let mut file = FatFile(fat12::find_file(&fat12::to_8_3("BADAPPLE.SND")?).await.ok()?);
    let mut head = [0u8; adpcm::HEADER_LEN];
    if file.read_at(0, &mut head).await < head.len() {
        return None;
    }
    Some((file, SoundHeader::parse(&head)?))
}

/// Missing or unreadable assets are unrecoverable -- blink red forever so the
/// failure is visible without a debugger attached.
async fn fail(led: &mut Output<'static>) -> ! {
    defmt::error!("assets missing or unreadable");
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(200)).await;
    }
}

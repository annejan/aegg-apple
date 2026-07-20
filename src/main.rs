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
mod usblog;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use defmt_rtt as _;

/// Panic handler that is visible without a debug probe.
///
/// `panic_probe` traps the CPU, which on a badge with nothing attached is
/// indistinguishable from a deadlock -- both look like "everything stopped".
/// Lighting the red LED separates the two, and that distinction has cost
/// several blind reflash cycles already.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Red LED is P1_07, active low: clearing the output drives it on. Done
    // through the PAC because a panic cannot borrow the GPIO driver.
    embassy_nrf::pac::P1.dirset().write(|w| w.0 = 1 << 7);
    embassy_nrf::pac::P1.outclr().write(|w| w.0 = 1 << 7);
    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

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
const LUT_SPEED: u8 = 8;

/// Backstop for a single panel refresh. Longer than the slowest full
/// waveform (~6 s) so it only fires on a genuinely stuck controller.
const REFRESH_TIMEOUT: Duration = Duration::from_secs(30);

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

/// Proof-of-life, independent of both the player and the audio task.
///
/// If these lines keep coming while everything else has gone quiet, the
/// executor is still scheduling and something is blocked on an await. If they
/// stop too, the whole executor is wedged. That distinction is not otherwise
/// observable on a badge with no debug probe.
#[embassy_executor::task]
async fn heartbeat_task() {
    let mut n: u32 = 0;
    loop {
        Timer::after(Duration::from_millis(500)).await;
        n += 1;
        crate::log!("hb {} (samples {}, busy {})", n, audio::samples_played(), crate::epd::busy_level() as u8);
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

    // USB serial first, before anything that can hang or panic.
    //
    // This badge has no debug probe on it, so `defmt` goes nowhere in the
    // field and the three LEDs are the entire diagnostic surface. Bringing
    // CDC-ACM up here means flash init, panel detection and header parsing --
    // the parts most likely to be wrong -- are all inside the window where
    // logging works. `usblog::run` starts HFXO itself (USBD cannot enumerate
    // on the internal RC) and returns false rather than blocking if the
    // crystal is dead.
    let usb_ok = usblog::run(p.USBD, &spawner).await;

    // Enumeration, driver bind and the host opening the port take a moment,
    // and anything logged before that is dropped -- the queue deliberately
    // discards lines while no terminal is attached. Two seconds is enough for
    // udev + a terminal that is already waiting on /dev/ttyACM0.
    if usb_ok {
        Timer::after(Duration::from_secs(2)).await;
    }
    crate::log!("aegg-apple: Bad Apple!! player starting");

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
    crate::log!("flash: QSPI up, USB log {}", if usb_ok { "on" } else { "off" });

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
    // The parsed headers, in full. A wrong frame count, fps or sample rate
    // here explains any amount of downstream weirdness, and until now there
    // was no way to see them on a probe-less badge.
    crate::log!(
        "video: {}x{}, {} frames @ {} fps",
        video_header.width,
        video_header.height,
        video_header.frame_count,
        video_header.fps
    );
    crate::log!(
        "sound: {} samples @ {} Hz, {} blocks of {} samples / {} bytes",
        sound_header.sample_count,
        sound_header.sample_rate,
        sound_header.block_count(),
        sound_header.block_samples,
        sound_header.block_bytes
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

    // Deliberately no clearing full refresh. It costs ~12 s of flashing
    // before a single frame appears, and the ghosting it would clear is
    // wanted here -- the artifacting is part of the look.
    crate::log!("panel: init done, busy={}, starting audio", epd::busy_level() as u8);
    led_blue.set_high();

    spawner.must_spawn(heartbeat_task());
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
    crate::log!("stopped: {} ({})", stop as u8, stop.name());
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

impl Stop {
    /// Human-readable form of the blink count, for the USB log.
    fn name(self) -> &'static str {
        match self {
            Stop::Complete => "complete",
            Stop::Fire => "fire held",
            Stop::AudioEnded => "audio ended",
            Stop::ReadFailed => "read failed",
            Stop::DecodeFailed => "decode failed",
        }
    }
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

    // Heartbeat for the audio clock. The whole frame-selection scheme rests on
    // `samples_played()` advancing; if it sticks, the video freezes on one
    // frame and there is otherwise no way to tell that from a stuck panel.
    let mut next_tick = Instant::now();

    loop {
        if Instant::now() >= next_tick {
            // Rescheduled from *now*, not from the previous deadline: a panel
            // refresh can hold the loop for seconds, and a fixed cadence would
            // then fire the heartbeat on every iteration to catch up.
            next_tick = Instant::now() + Duration::from_secs(1);
            crate::log!(
                "tick: samples {}, shown {}, skipped {}, stalled {}",
                audio::samples_played(),
                shown.map(|f| f as i64).unwrap_or(-1),
                skipped,
                stalled
            );
        }

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
                crate::log!("no audio after 1.5 s; running video on the wall clock");
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

        crate::log!("f{}: reading", target);
        let Some(len) = read_frame(&mut file, &header, target, &mut encoded).await else {
            defmt::warn!("frame {} unreadable", target);
            crate::log!("frame {} unreadable", target);
            return Stop::ReadFailed;
        };

        if video::decode_frame(&encoded[..len], &mut plane).is_none() {
            defmt::warn!("frame {} did not decode", target);
            crate::log!("frame {} did not decode ({} coded bytes)", target, len);
            return Stop::DecodeFailed;
        }

        // Green toggles before the refresh and blue after it, so the pair
        // distinguishes "never reached the panel" from "the panel refresh
        // never returned" without a debug probe.
        led_green.toggle();
        crate::log!("f{}: decoded {} B, busy={}, refreshing", target, len, epd::busy_level() as u8);
        // Outer backstop. The driver's own BUSY waits are bounded, but this
        // guarantees the loop keeps running whatever the panel does -- a
        // stalled refresh costs one frame, not the whole video.
        let refresh_started = Instant::now();
        let timed_out = with_timeout(REFRESH_TIMEOUT, panel.show(&plane, LUT_SPEED))
            .await
            .is_err();
        let elapsed_ms = refresh_started.elapsed().as_millis();
        if timed_out {
            stalled += 1;
            defmt::warn!("refresh timed out on frame {} ({} so far)", target, stalled);
        }
        led_blue.toggle();

        defmt::info!("frame {} ({} skipped)", target, skipped);
        // One line per shown frame: index, coded size, how long the panel
        // actually took, and whether the backstop fired. This is the record
        // that says whether the panel or the clock is the thing going wrong.
        crate::log!(
            "frame {}: {} B, show {} ms{}, skipped {}",
            target,
            len,
            elapsed_ms,
            if timed_out { " TIMEOUT" } else { "" },
            skipped
        );
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
    crate::log!("FATAL: assets missing or unreadable");
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(200)).await;
    }
}

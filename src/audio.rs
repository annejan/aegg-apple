//! ADPCM playback out of the piezo buzzer, and the clock everything else
//! follows.
//!
//! The buzzer on P0_13 has no DAC behind it, so samples are played by
//! modulating the PWM duty cycle: the counter top is set so that one PWM
//! period is exactly one sample period, and each sample becomes a duty value
//! fed to the peripheral by EasyDMA.
//!
//! Two buffers are handed to the PWM sequencer in `Infinite` mode, which
//! plays seq0, seq1, seq0, ... and re-reads the DMA pointer at each sequence
//! start. Whenever one buffer starts playing, the other is idle and gets
//! refilled. That is the whole design; the awkward part is that the hardware
//! is reading a buffer the code also writes, which the borrow checker cannot
//! express -- see `Buffers` below.
//!
//! Audio is the master clock. It is the one thing here that must not stutter:
//! a late video frame is invisible, a late sample is an audible click. The
//! video side reads [`samples_played`] and picks whichever frame matches,
//! skipping any it could not keep up with.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_nrf::Peri;
use embassy_nrf::gpio::Pin;
use embassy_nrf::pwm::{
    Config, Prescaler, Sequence, SequenceConfig, SequenceLoad, SequenceMode, SequencePwm,
    Sequencer, StartSequence,
};
use embassy_time::{Duration, Timer};

use crate::adpcm::{self, SoundHeader};
use crate::asset::AssetRead;

/// Samples per DMA buffer. At 6 kHz this is ~85 ms, so the refill poll below
/// has an order of magnitude more slack than it needs even while the e-paper
/// panel is mid-refresh.
const CHUNK: usize = 512;

/// Largest ADPCM block the decoder will accept, in samples and in bytes.
/// The encoder emits 1017-sample, 512-byte blocks.
const MAX_BLOCK_SAMPLES: usize = 1024;
const MAX_BLOCK_BYTES: usize = 1024;

/// The nRF52840 PWM base clock, with `Prescaler::Div1`.
const PWM_CLOCK_HZ: u32 = 16_000_000;

/// Largest counter top we will accept before trading resolution for carrier
/// frequency. 320 puts the carrier at 50 kHz or above -- inaudible -- while
/// still leaving over eight bits of duty resolution.
const MAX_TOP: u32 = 320;

/// Choose a counter top and hold count for a sample rate.
///
/// Returns `(top, refresh)` such that `top * (refresh + 1)` PWM ticks make up
/// one sample period. `refresh` is the number of *extra* periods each sample
/// is held for, so the sample rate is `PWM_CLOCK_HZ / (top * (refresh + 1))`.
fn carrier_for(sample_rate: u32) -> (u16, u32) {
    let rate = sample_rate.max(1);
    let mut refresh = 0u32;
    loop {
        let top = PWM_CLOCK_HZ / (rate * (refresh + 1));
        if top <= MAX_TOP || refresh >= 15 {
            // Never return a top of 0, which would stop the counter.
            return (top.max(2) as u16, refresh);
        }
        refresh += 1;
    }
}

/// Total samples handed to the PWM peripheral since playback began.
static SAMPLES_PLAYED: AtomicU32 = AtomicU32::new(0);

/// Set once the audio stream has run out.
static FINISHED: AtomicBool = AtomicBool::new(false);

/// How many times the track has restarted.
static LOOPS: AtomicU32 = AtomicU32::new(0);

/// Number of completed passes through the track.
pub fn loops() -> u32 {
    LOOPS.load(Ordering::Relaxed)
}

/// How many samples have been queued for playback so far.
///
/// This lags what has physically left the pin by up to one buffer (~85 ms),
/// which is well under one frame at 4 fps and so does not affect frame
/// selection.
pub fn samples_played() -> u32 {
    SAMPLES_PLAYED.load(Ordering::Relaxed)
}

/// Whether the audio stream has been played to the end.
///
/// Unused while the track loops forever, but kept: it is the only signal a
/// caller has that playback ended.
#[allow(dead_code)]
pub fn finished() -> bool {
    FINISHED.load(Ordering::Relaxed)
}

/// The two DMA buffers.
///
/// The PWM peripheral holds a pointer to each of these for as long as
/// playback runs, while the refill loop writes whichever one is not currently
/// being read. Rust cannot check that alternation, so the buffers live behind
/// `UnsafeCell` and the safety argument is the hardware's: `Infinite` mode
/// plays the two sequences strictly alternately, and a buffer is only written
/// after the peripheral has signalled that it started playing the *other*
/// one.
struct Buffers {
    words: [UnsafeCell<[u16; CHUNK]>; 2],
}

// SAFETY: the only writer is the single audio task, and the only reader is
// the PWM peripheral's DMA, which never reads the buffer being written.
unsafe impl Sync for Buffers {}

static BUFFERS: Buffers = Buffers {
    words: [UnsafeCell::new([0; CHUNK]), UnsafeCell::new([0; CHUNK])],
};

/// Pulls decoded samples out of the ADPCM stream one block at a time.
struct Stream<R: AssetRead> {
    reader: R,
    header: SoundHeader,
    /// The most recently decoded block.
    samples: [i16; MAX_BLOCK_SAMPLES],
    /// How many of `samples` are valid.
    valid: usize,
    /// Read cursor within `samples`.
    cursor: usize,
    /// Index of the next block to decode.
    next_block: u32,
    done: bool,
}

impl<R: AssetRead> Stream<R> {
    fn new(reader: R, header: SoundHeader) -> Self {
        Self {
            reader,
            header,
            samples: [0; MAX_BLOCK_SAMPLES],
            valid: 0,
            cursor: 0,
            next_block: 0,
            done: false,
        }
    }

    /// Decode the next block into `samples`, returning false at end of stream.
    async fn refill(&mut self) -> bool {
        if self.next_block >= self.header.block_count() {
            self.done = true;
            return false;
        }

        let len = self.header.block_bytes as usize;
        let mut raw = [0u8; MAX_BLOCK_BYTES];
        let n = self
            .reader
            .read_at(self.header.block_offset(self.next_block), &mut raw[..len])
            .await;
        if n < len {
            self.done = true;
            return false;
        }

        let want = (self.header.block_samples as usize).min(MAX_BLOCK_SAMPLES);
        self.valid = adpcm::decode_block(&raw[..len], &mut self.samples[..want]);
        self.cursor = 0;
        self.next_block += 1;
        self.valid > 0
    }

    /// Rewind to the first block. Used to loop the track.
    fn rewind(&mut self) {
        self.next_block = 0;
        self.cursor = 0;
        self.valid = 0;
        self.done = false;
    }

    /// Next sample, or `None` once the stream is exhausted.
    async fn next(&mut self) -> Option<i16> {
        while self.cursor >= self.valid {
            if self.done || !self.refill().await {
                return None;
            }
        }
        let s = self.samples[self.cursor];
        self.cursor += 1;
        Some(s)
    }
}

/// Play `BADAPPLE.SND` to the buzzer until it ends.
///
/// `pwm` must be PWM0 and `pin` the buzzer pin. Returns when the stream is
/// finished; the caller is expected to keep the task alive or stop.
pub async fn play<R: AssetRead>(
    pwm: Peri<'static, embassy_nrf::peripherals::PWM0>,
    pin: Peri<'static, impl Pin>,
    reader: R,
    header: SoundHeader,
) {
    // Split the sample period into a fast PWM carrier plus a hold count.
    //
    // The obvious arrangement -- one PWM period per sample -- puts the
    // carrier *at* the sample rate, so the pin emits a 6 kHz square wave
    // whose duty wobbles. That is audible as a loud tone, not as music. The
    // carrier has to sit above hearing and each sample must be held for
    // several periods, which is what `refresh` does.
    //
    // At 6 kHz this settles on top = 296 (a 54 kHz carrier) held for 9
    // periods, giving 6006 Hz -- 0.1% fast, and since audio is the master
    // clock the video simply follows it.
    let (top, refresh) = carrier_for(header.sample_rate);
    defmt::info!(
        "audio: {} Hz, carrier {} Hz, top {}, refresh {}",
        PWM_CLOCK_HZ / (top as u32 * (refresh + 1)),
        PWM_CLOCK_HZ / top as u32,
        top,
        refresh
    );
    crate::log!(
        "audio: {} Hz, carrier {} Hz, top {}, refresh {}",
        PWM_CLOCK_HZ / (top as u32 * (refresh + 1)),
        PWM_CLOCK_HZ / top as u32,
        top,
        refresh
    );

    let mut config = Config::default();
    config.prescaler = Prescaler::Div1;
    config.max_duty = top;
    config.sequence_load = SequenceLoad::Common;

    let mut pwm = match SequencePwm::new_1ch(pwm, pin, config) {
        Ok(pwm) => pwm,
        Err(_) => {
            defmt::error!("audio: PWM init failed");
            crate::log!("audio: PWM init failed");
            return;
        }
    };

    let mut stream = Stream::new(reader, header);

    // Prime both buffers before starting, so the first thing the peripheral
    // reads is real audio rather than whatever the statics held.
    let silence = adpcm::sample_to_duty(0, top);
    let mut ended = false;
    for half in 0..2 {
        ended |= !fill(half, &mut stream, top, silence).await;
    }

    // SAFETY: the peripheral reads these for as long as the `Sequencer`
    // lives, and the refill loop below only writes the half that is not
    // playing. See `Buffers`.
    let (words0, words1) = unsafe {
        (
            &*(BUFFERS.words[0].get() as *const [u16; CHUNK]),
            &*(BUFFERS.words[1].get() as *const [u16; CHUNK]),
        )
    };

    // No gap between the two buffers -- any end_delay is an audible click at
    // every buffer boundary.
    let mut seq_config = SequenceConfig::default();
    seq_config.refresh = refresh;
    seq_config.end_delay = 0;
    let sequencer = Sequencer::new(
        &mut pwm,
        Sequence::new(words0.as_slice(), seq_config.clone()),
        Some(Sequence::new(words1.as_slice(), seq_config)),
    );

    if sequencer
        .start(StartSequence::Zero, SequenceMode::Infinite)
        .is_err()
    {
        defmt::error!("audio: sequencer start failed");
        crate::log!("audio: sequencer start failed");
        return;
    }

    // Which half the peripheral was last seen to start. Buffer 0 is playing
    // first, so buffer 1 is the one that may be refilled.
    let mut playing = 0usize;
    clear_started();

    while !ended {
        // Poll rather than wait on an interrupt: the SEQSTARTED events are
        // exposed only as PPI endpoints, and at 85 ms per buffer a 5 ms poll
        // is both cheap and enormously early.
        Timer::after(Duration::from_millis(5)).await;

        let started = take_started();
        if started[0] && playing != 0 {
            playing = 0;
        } else if started[1] && playing != 1 {
            playing = 1;
        } else if !started[0] && !started[1] {
            continue;
        }

        SAMPLES_PLAYED.fetch_add(CHUNK as u32, Ordering::Relaxed);
        ended = !fill(1 - playing, &mut stream, top, silence).await;
    }

    // Let the tail drain, then park the pin low so the piezo is not left
    // holding a DC bias.
    Timer::after(Duration::from_millis(
        (CHUNK as u64 * 2 * 1000) / header.sample_rate as u64,
    ))
    .await;
    sequencer.stop();
    FINISHED.store(true, Ordering::Relaxed);
}

/// Fill one buffer from the stream. Returns false once the stream has ended,
/// after padding the remainder with silence.
async fn fill<R: AssetRead>(
    half: usize,
    stream: &mut Stream<R>,
    top: u16,
    silence: u16,
) -> bool {
    // SAFETY: `half` is the buffer the peripheral is not currently reading.
    let buf = unsafe { &mut *BUFFERS.words[half].get() };

    for slot in buf.iter_mut() {
        let sample = match stream.next().await {
            Some(s) => Some(s),
            None => {
                // End of track: start it again and carry straight on, so the
                // loop has no gap in it. The sample counter restarts too --
                // it is the video's clock, and resetting it is what makes the
                // picture wrap back to frame 0 with the music instead of
                // drifting a little further out on every pass.
                stream.rewind();
                SAMPLES_PLAYED.store(0, Ordering::Relaxed);
                LOOPS.fetch_add(1, Ordering::Relaxed);
                stream.next().await
            }
        };
        *slot = match sample {
            Some(s) => adpcm::sample_to_duty(s, top),
            // Only reachable if the file itself is unreadable.
            None => silence,
        };
    }

    !stream.done
}

/// Read and clear the two SEQSTARTED events.
fn take_started() -> [bool; 2] {
    let r = embassy_nrf::pac::PWM0;
    let mut out = [false; 2];
    for (i, flag) in out.iter_mut().enumerate() {
        let ev = r.events_seqstarted(i);
        if ev.read() != 0 {
            ev.write_value(0);
            *flag = true;
        }
    }
    out
}

fn clear_started() {
    let r = embassy_nrf::pac::PWM0;
    for i in 0..2 {
        r.events_seqstarted(i).write_value(0);
    }
}

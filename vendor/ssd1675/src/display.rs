use crate::{
    command::{BufCommand, Command, DeepSleepMode, DisplayUpdateSequenceOption},
    config::Config,
    interface::DisplayInterface,
};

/// Lowest ambient temperature (°C) covered by the per-temperature LUT lookup
/// table.  Below this, the driver clamps to entry 0.  See
/// [`Display::set_active_temperature`].
pub const LUT_TABLE_MIN_C: i16 = -10;

/// Width of one LUT-table band in °C × 10.  Matches the deployed panels'
/// 4 °C OTP TR-band granularity (datasheet §6.9, pg 17 example).
pub const LUT_TABLE_STEP_C10: u16 = 40;

/// Number of LUT-table entries.  16 × 4 °C = 64 °C span covering
/// `LUT_TABLE_MIN_C` (−10 °C) through +54 °C — badge's plausible operating
/// range plus headroom.  Each entry is a 107-byte OTP WS image probed at
/// boot via cmd `0x33` with the temperature register set to the
/// corresponding band centre; see `fw::epd::probe_lut_table`.
pub const LUT_TABLE_SIZE: usize = 16;

/// Pre-computed °C × 10 lower bound — saves runtime casting in band lookup.
const LUT_TABLE_MIN_C10: i32 = (LUT_TABLE_MIN_C as i32) * 10;

/// Default active temperature (°C × 10) used by `Display::new` until the
/// caller calls `set_active_temperature`.  20 °C — typical indoor ambient.
const DEFAULT_ACTIVE_TEMP_C10: i16 = 200;

/// Upper bound on how long [`Display::reset`] waits for an in-flight drive
/// waveform to finish before pulsing RES#.  A full refresh is ~2-4 s at room
/// temperature and stretches when the panel is cold (badges live outdoors), so
/// this is a generous backstop — under normal operation the wave completes long
/// before it and the timeout never elapses.  If it ever does (wedged BUSY), the
/// old abort-and-reset behaviour resumes.
const RESET_ACTIVATION_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(12);

/// Waveform update mode passed to `update_bw()`.
#[derive(Clone, Copy, PartialEq)]
pub enum UpdateMode {
    /// Mode 1 (0xC7): uses the LUT register directly, no OTP reload.
    Mode1,
    /// Mode 2 (0xCF): reloads the full OTP waveform from the controller.
    Mode2,
}

/// Zero out the inversion / pre-charge phases of an OTP-probed LUT.
///
/// The OTP waveform contains a few timing phases (groups 0–2 on SSD1675's
/// 7-row layout; phases 0–14 of the 50-byte timing region on SSD1675B's
/// 10-row layout) that drive every pixel through an inversion / erase cycle
/// before settling on the target colour.  Visible as a brief flash on every
/// refresh.  Patching those phases to zero (with a small compensating tweak
/// on SSD1675 so neighbour cells don't get under-driven) gives a
/// flicker-free fast refresh at the cost of accumulating ghosting from
/// prior frames — fine for menu / text screens that get periodic
/// full-waveform refreshes via `update_tc`.
///
/// Caller is responsible for deciding which buffer gets patched — typically
/// the fast-refresh table that backs [`Display::update_bw`], leaving the
/// full-waveform table (used by [`Display::update_tc`]) untouched.
pub fn patch_no_invert(lut: &mut [u8; 107], variant: DisplayVariant) {
    match variant {
        DisplayVariant::Ssd1675 => {
            // 7-byte rows: 5 LUTs × 7 phases = 35 waveform bytes (0..34);
            // 7 TPs × 5 bytes = 35 timing bytes (35..69); 6-byte trailer.
            // LUT row R phase P lives at byte R*7 + P.

            // ── White-wipe re-tuning of phases 0/1/2 ───────────────────
            // Repurpose the OTP's inversion / erase phases (visible
            // flash + a source of fringe-field blooming) into a uniform
            // white-wipe.  Voltage `0x80` = sub-A code 10 = VSH2 =
            // panel's white-drive level.  All four BW LUT rows get the
            // same value, eliminating horizontal voltage gradients
            // between adjacent pixels — per PMC6187556 the gradient is
            // what drives the lateral particle migration that shows up
            // as ink blooming on enclosed counters (text holes etc.).
            // VCOM idle (LUT4 = 0) throughout.
            for p in 0..3 {
                lut[p] = 0x80;        // LUT0 phase p
                lut[7 + p] = 0x80;    // LUT1 phase p
                lut[14 + p] = 0x80;   // LUT2 phase p
                lut[21 + p] = 0x80;   // LUT3 phase p
                lut[28 + p] = 0x00;   // LUT4 phase p — VCOM idle
            }

            // Scale TP0..TP2 timing to 30% of the OTP budget — stronger
            // white pull than the earlier 22%, still well under the full
            // inversion-flash length.  Non-zero values floor to 1 so the
            // chip still executes the phase (timing 0 skips).
            for b in &mut lut[35..50] {
                if *b != 0 {
                    *b = ((*b as u16 * 3 / 10).max(1)) as u8;
                }
            }

            // ── Active drive phases TP3 / TP4 ──────────────────────────
            // The pre-existing no-invert compensation: tweak the OTP's
            // TP3 / TP4 timings so the post-wipe drive lands cleanly on
            // the target colour.  Leaves the OTP-probed voltage bytes
            // for phases 3 and 4 (bytes [3, 10, 17, 24, 31] and
            // [4, 11, 18, 25, 32]) untouched — those carry the
            // per-temperature drive recipe.
            lut[51] = 2;
            lut[52] = lut[52].wrapping_add(0x10);
            lut[53] = 2;
            lut[54] = lut[54].wrapping_add(0x10);
            lut[55] = 2;

            // Shorten LUT0's black-drive sub-frames in the active content
            // phases (3 / 4) by ~25%.  TP timing is shared across LUT
            // rows so we can't scale only the black row via TPx; instead
            // we zero out sub-D (bits[1:0]) of LUT0's voltage byte —
            // that sub-frame becomes a 0V hold for black-target pixels
            // (LUT0 = RAM1=0,RAM2=0) while LUT1 (white target) is left
            // intact.  Reduces over-drive on black ink without affecting
            // white-drive duration.  Closest single-bit-pattern step to
            // "20% shorter" given the 4-sub-frame granularity.
            lut[3] &= 0xFC; // LUT0 phase 3 — drop sub-D drive
            lut[4] &= 0xFC; // LUT0 phase 4 — drop sub-D drive

            // ── White-stabilize tail at phase 5 ────────────────────────
            // 20 frames of VSH2 drive on the white-target LUT rows only
            // (LUT1, LUT3).  Black-target rows (LUT0, LUT2) stay at 0V
            // so black content isn't lightened.  Re-seats white-particle
            // ink that drifted during the main drive — further reduces
            // bloom on top of the front-of-frame white wipe above.
            lut[5] = 0x00;   // LUT0 phase 5 — no drive on black-target
            lut[12] = 0x80;  // LUT1 phase 5 — white drive
            lut[19] = 0x00;  // LUT2 phase 5 — no drive
            lut[26] = 0x80;  // LUT3 phase 5 — white drive
            lut[33] = 0x00;  // LUT4 phase 5 — VCOM idle
            lut[60] = 0x16;  // TP5-A — 22 frames (was 20, +10% white-stabilize)
            lut[61] = 0;
            lut[62] = 0;
            lut[63] = 0;
            lut[64] = 0;     // TP5-RP — no repeat
        }
        DisplayVariant::Ssd1675B => {
            // 10-byte rows: waveform 0–49, TP 50–99 (10 groups × {A,B,C,D,RP}).
            // Strip the inversion/erase phases (0–2)…
            lut[50..64].fill(0);
            // …and cap the repeat count (RP byte = group base + 4) to a single
            // execution on every phase.  The raw OTP repeats drive phases
            // many times (e.g. RP=13 → 14×), cycling black↔white = a strobe on
            // partial refreshes.  RP=0 → one pass per phase, no strobe.
            // (Raise toward 1–2 if delta contrast drops too far.)
            for g in 0..10 {
                let rp = 50 + g * 5 + 4;
                if rp < lut.len() {
                    lut[rp] = 0;
                }
            }
        }
    }
}

/// Best-effort guess of the controller variant from a 107-byte LUT-register
/// readback.  Two signals:
///   * bytes 7..=9 — SSD1675's WS row index 1 columns A–C are non-zero in
///     7-byte-row layout; the same byte positions sit inside row 0 of
///     SSD1675B's 10-byte-row layout where they read back 0x00.
///   * byte 100 — SSD1675B's VGH register; outside the WS on SSD1675 (reads
///     back 0x00).
/// Requires both signals to agree before declaring B, to avoid
/// mis-classifying a panel with sparse OTP.  Falls back to the row-layout
/// signal on disagreement.
///
/// Caller should prefer `Display::set_variant` when the panel identity is
/// known out-of-band; this heuristic exists for the boot-probe path where
/// the variant isn't yet known.
pub fn detect_variant_from_otp(otp: &[u8; 107]) -> DisplayVariant {
    let row_signal_b = otp[7] == 0 && otp[8] == 0 && otp[9] == 0;
    let trailer_signal_b = otp[100] != 0;
    if row_signal_b && trailer_signal_b {
        DisplayVariant::Ssd1675B
    } else if !row_signal_b && !trailer_signal_b {
        DisplayVariant::Ssd1675
    } else if row_signal_b {
        DisplayVariant::Ssd1675B
    } else {
        DisplayVariant::Ssd1675
    }
}

/// Display controller variant.
///
/// The SSD1675B uses a 10-byte-per-row LUT format (100-byte LUT region:
/// 5 LUTs × 10 phases + 10 TP/RP groups × 5 bytes; voltage trailer at
/// bytes 100..=106).  The SSD1675 uses a 7-byte-per-row LUT format
/// (70-byte LUT region: 5 LUTs × 7 phases + 7 TP/RP groups × 5 bytes;
/// voltage trailer at bytes 70..=75).  Total OTP slot is 107 bytes for
/// SSD1675B and 76 bytes for SSD1675 (the latter padded to 107 by the
/// register-0x33 readback).
#[derive(Clone, Copy, PartialEq)]
pub enum DisplayVariant {
    /// 10-byte-per-row LUT format. 100-byte LUT (cmd 0x32);
    /// voltage trailer in OTP bytes 100..=106.
    Ssd1675B,
    /// 7-byte-per-row LUT format. 70-byte LUT (cmd 0x32);
    /// voltage trailer in OTP bytes 70..=75.
    Ssd1675,
}


// Max display resolution is 176x296
/// The maximum number of rows supported by the controller
pub const MAX_GATE_OUTPUTS: u16 = 296;
/// The maximum number of columns supported by the controller
pub const MAX_SOURCE_OUTPUTS: u8 = 176;

/// Default LUT-cycle scale for SSD1675 (1675A) panels.
/// `100` = OEM duration. Tune after panel calibration.
pub const DEFAULT_LUT_SPEED_SSD1675: u8 = 100;
/// Default LUT-cycle scale for SSD1675B panels.
/// `100` = OEM duration. Tune after panel calibration.
pub const DEFAULT_LUT_SPEED_SSD1675B: u8 = 100;

/// Drive-voltage / timing register set pushed before every refresh (cmd
/// 0x03/0x04/0x2C/0x3A/0x3B), replacing the per-band OTP trailer read.
/// Per-band values live in [`crate::b_on_a::B_ON_A_VOLTAGES`].
#[derive(Clone, Copy)]
pub struct VoltageProfile {
    /// cmd 0x03 — gate driving voltage (VGH).
    pub vgh: u8,
    /// cmd 0x04 — source driving voltages.
    pub vsh1: u8,
    pub vsh2: u8,
    pub vsl: u8,
    /// cmd 0x2C — VCOM.
    pub vcom: u8,
    /// cmd 0x3A — dummy line period.
    pub dummy: u8,
    /// cmd 0x3B — gate line width.
    pub gate: u8,
}


impl DisplayVariant {
    /// Number of bytes pushed to the controller's LUT register (cmd 0x32).
    /// Matches prior shipped behavior — voltage trailer starts at byte
    /// 100 (1675B) / 70 (1675A) inside the 107-byte OTP image.
    pub fn lut_byte_len(self) -> usize {
        match self {
            DisplayVariant::Ssd1675  => 70, // 5×7 waveform + 7×5 timing
            DisplayVariant::Ssd1675B => 99, // 5×10 waveform + ~50 timing (last OTP byte unused)
        }
    }

    /// Half-open `[start, end)` byte range inside the 107-byte OTP LUT
    /// covering the timing-cycle bytes (durations + repeats). Bytes here
    /// are scaled by [`Display::set_lut_speed`] before each refresh.
    pub fn timing_range(self) -> core::ops::Range<usize> {
        match self {
            DisplayVariant::Ssd1675  => 35..70,
            DisplayVariant::Ssd1675B => 50..100,
        }
    }

    /// Per-variant default for [`Display::set_lut_speed`]. Applied
    /// automatically when the variant is set or detected.
    pub fn default_lut_speed(self) -> u8 {
        match self {
            DisplayVariant::Ssd1675  => DEFAULT_LUT_SPEED_SSD1675,
            DisplayVariant::Ssd1675B => DEFAULT_LUT_SPEED_SSD1675B,
        }
    }
}

/// Copy `src` into `out` and scale every non-zero byte in the LUT timing
/// region by `scale / 100`. Returns the byte count copied. Zero bytes
/// (e.g. a deliberately zeroed inversion phase) stay zero.
fn scale_lut_into(out: &mut [u8; 107], src: &[u8], variant: DisplayVariant, scale: u8) -> usize {
    let n = src.len().min(out.len());
    out[..n].copy_from_slice(&src[..n]);
    let r = variant.timing_range();
    for b in &mut out[r.start..r.end.min(n)] {
        if *b != 0 {
            *b = ((*b as u16 * scale as u16) / 100).min(u8::MAX as u16) as u8;
        }
    }
    n
}

/// Total panel frames a LUT's timing region schedules: for each phase,
/// `(TPA + TPB + TPC + TPD) × (RP + 1)`, summed over all phases.  Multiply
/// by the panel frame period (~1 frame ≈ a few ms) to estimate the
/// wall-clock waveform duration.  Diagnostic aid for tuning the refresh
/// budget — does NOT include RAM-upload or busy-poll overhead.
pub fn waveform_frames(lut: &[u8], variant: DisplayVariant) -> u32 {
    let r = variant.timing_range();
    let mut frames = 0u32;
    let mut tp = r.start;
    while tp + 5 <= r.end && tp + 5 <= lut.len() {
        let sub = lut[tp] as u32 + lut[tp + 1] as u32 + lut[tp + 2] as u32 + lut[tp + 3] as u32;
        let rp = lut[tp + 4] as u32 + 1; // RP=0 means execute once
        frames += sub * rp;
        tp += 5;
    }
    frames
}

// Magic numbers from the data sheet
const ANALOG_BLOCK_CONTROL_MAGIC: u8 = 0x54;
const DIGITAL_BLOCK_CONTROL_MAGIC: u8 = 0x3B;

/// Represents the dimensions of the display.
pub struct Dimensions {
    /// The number of rows the display has.
    ///
    /// Must be less than or equal to MAX_GATE_OUTPUTS.
    pub rows: u16,
    /// The number of columns the display has.
    ///
    /// Must be less than or equal to MAX_SOURCE_OUTPUTS.
    pub cols: u8,
}

/// Represents the physical rotation of the display relative to the native orientation.
#[derive(Clone, Copy)]
pub enum Rotation {
    Rotate0,
    Rotate90,
    Rotate180,
    Rotate270,
}

impl Default for Rotation {
    fn default() -> Self {
        Rotation::Rotate0
    }
}

/// A configured display with a hardware interface.
pub struct Display<'a, I>
where
    I: DisplayInterface,
{
    interface: I,
    config: Config<'a>,
    /// Per-temperature LUT lookup table — full OTP waveform with inversion
    /// phases intact.  Used by [`update_tc`] for tri-color full refreshes
    /// (Name screen, sponsor slideshow) where the inversion phases are
    /// needed to fully erase ghosting and re-seat red ink.
    lut_table: Option<&'static [[u8; 107]; LUT_TABLE_SIZE]>,
    /// Same table with inversion phases zeroed out per [`patch_no_invert`].
    /// Used by [`update_bw`] for flicker-free fast refreshes (every other
    /// screen).  Without this, every BW refresh drives the OTP's pre-charge
    /// / erase phases too and visibly inverts the panel.
    lut_table_no_invert: Option<&'static [[u8; 107]; LUT_TABLE_SIZE]>,
    /// Temperature-independent full-refresh waveform that replaces
    /// `lut_table` in [`update_tc`] when set (see
    /// [`Display::set_full_lut_override`]).  [`Display::update_tc_otp`]
    /// ignores it, so a caller can still reach the panel's own probed OTP
    /// waveform.  The partial / no-invert path never consults it.
    full_lut_override: Option<&'static [u8; 107]>,
    /// Most recently set ambient temperature in °C × 10.  Indexes
    /// `lut_table` on every refresh.  Defaults to 20 °C if
    /// [`set_active_temperature`] is never called.
    active_temp_c10: i16,
    /// Display controller variant.  Default `Ssd1675B`; caller should
    /// override via [`set_variant`] when known out-of-band, since the
    /// `init()` sequence is variant-gated (cmd `0x18` only sent on B).
    pub(crate) variant: DisplayVariant,
    /// True once `set_variant` has been called by the caller.
    variant_explicit: bool,
    /// True between sending `UpdateDisplay` (0x20, master activation) and
    /// observing the following `busy_wait_for_completion` return — i.e. while
    /// the panel *may* be autonomously running a drive waveform whose
    /// completion this host has not yet seen.  A future dropped (async
    /// cancellation) or an SPI error between those two points leaves this
    /// `true`.  [`reset`] consults it to avoid pulsing RES# mid-waveform,
    /// which would freeze the bistable ink mid-phase (on SSD1675B, mid the
    /// full LUT's early inversion phases → the whole image latches inverted).
    ///
    /// Keyed on this flag rather than the BUSY pin alone because Deep Sleep
    /// (0x10) also holds BUSY high indefinitely (exit requires HW reset), so
    /// BUSY-high cannot by itself distinguish "waveform in flight" from
    /// "asleep": an unconditional wait-BUSY-low before reset would stall
    /// forever after every deep_sleep.  The flag is always cleared before a
    /// successful refresh returns, so deep_sleep is only ever reached with it
    /// `false`.
    activation_pending: bool,
}

impl<'a, I> Display<'a, I>
where
    I: DisplayInterface,
{
    pub fn new(interface: I, config: Config<'a>) -> Self {
        Self {
            interface,
            config,
            lut_table: None,
            lut_table_no_invert: None,
            full_lut_override: None,
            active_temp_c10: DEFAULT_ACTIVE_TEMP_C10,
            variant: DisplayVariant::Ssd1675B,
            variant_explicit: false,
            activation_pending: false,
        }
    }

    /// Fire master activation (`UpdateDisplay`, 0x20) and wait for the drive
    /// waveform to finish.  Brackets the wait with [`activation_pending`] so a
    /// dropped future or SPI error between the trigger and observed completion
    /// is remembered — [`reset`] then waits the waveform out instead of
    /// aborting it with a mid-phase RES# pulse.
    async fn trigger_and_wait(&mut self) -> Result<(), I::Error> {
        Command::UpdateDisplay.execute(&mut self.interface).await?;
        // Set AFTER the 0x20 byte has left the bus: before this point nothing
        // is running, so an earlier drop needs no guard.
        self.activation_pending = true;
        self.interface.busy_wait_for_completion().await?;
        self.activation_pending = false;
        Ok(())
    }

    /// Register the boot-probed per-temperature LUT tables.  Both must be
    /// allocated in `'static` storage (e.g. via `StaticCell`).
    ///
    /// `full` is the raw OTP waveform with inversion phases — used by
    /// [`update_tc`] for tri-color full refreshes.
    ///
    /// `no_invert` is the same table with inversion phases zeroed (see
    /// [`patch_no_invert`]) — used by [`update_bw`] for flicker-free
    /// fast refreshes.
    ///
    /// Until this is called, every refresh path panics — there's no
    /// fallback LUT.
    pub fn register_lut_tables(
        &mut self,
        full: &'static [[u8; 107]; LUT_TABLE_SIZE],
        no_invert: &'static [[u8; 107]; LUT_TABLE_SIZE],
    ) {
        self.lut_table = Some(full);
        self.lut_table_no_invert = Some(no_invert);
    }

    /// Install a temperature-independent full-refresh waveform that
    /// [`update_tc`](Self::update_tc) uses instead of the probed OTP band
    /// table — the hook for a hand-calibrated full LUT (its voltage trailer
    /// is pushed along with it).
    ///
    /// [`update_tc_otp`](Self::update_tc_otp) always bypasses the override
    /// and drives the panel's own probed OTP waveform, and the partial /
    /// no-invert path is untouched either way.  `None` clears it.
    pub fn set_full_lut_override(&mut self, lut: Option<&'static [u8; 107]>) {
        self.full_lut_override = lut;
    }

    /// Set the ambient temperature (°C × 10) used to index `lut_table` on
    /// the next refresh.  Caller is responsible for refreshing this with
    /// a reasonable approximation (e.g. the nRF52840 die sensor minus a
    /// self-heating bias).  Cheap — just an integer store.
    pub fn set_active_temperature(&mut self, c10: i16) {
        self.active_temp_c10 = c10;
    }

    /// Temperature band index for the current [`active_temp_c10`], clamped to
    /// the table.  Band i centres on `-10 + 4*i` °C.
    fn band_idx(&self) -> usize {
        let offset = self.active_temp_c10 as i32 - LUT_TABLE_MIN_C10;
        (offset / LUT_TABLE_STEP_C10 as i32).clamp(0, (LUT_TABLE_SIZE - 1) as i32) as usize
    }

    /// Pick the LUT entry whose 4 °C band contains the current temperature.
    fn select_from(&self, table: &'static [[u8; 107]; LUT_TABLE_SIZE]) -> &'static [u8; 107] {
        &table[self.band_idx()]
    }

    /// Full-refresh waveform: the calibrated override when one is installed
    /// (see [`set_full_lut_override`](Self::set_full_lut_override)),
    /// otherwise the panel's own probed OTP entry for the current band.
    fn select_full(&self) -> &'static [u8; 107] {
        self.full_lut_override.unwrap_or_else(|| self.otp_band())
    }

    /// The panel's **own probed OTP** entry for the current temperature band.
    /// Backs the partial / BW paths, whose waveforms are derived from it and
    /// therefore expect its voltage trailer.
    fn otp_band(&self) -> &'static [u8; 107] {
        let t = self.lut_table.expect("LUT tables must be registered before refresh");
        self.select_from(t)
    }

    /// Delta / partial waveform — inversion phases stripped (non-flashing).
    /// The panel's own probed `no_invert` table (patch_no_invert) for both
    /// variants, so partials don't blink like the raw OTP full waveform.
    fn select_no_invert(&self) -> &'static [u8; 107] {
        let t = self
            .lut_table_no_invert
            .expect("LUT tables must be registered before refresh");
        self.select_from(t)
    }

    /// Return the configured display controller variant.
    pub fn variant(&self) -> DisplayVariant {
        self.variant
    }

    /// Set the controller variant explicitly.  Recommended whenever the
    /// panel identity is known out-of-band — `init()` is variant-gated
    /// (cmd `0x18` only sent on B).  Call before `reset()` / refresh.
    pub fn set_variant(&mut self, variant: DisplayVariant) {
        self.variant = variant;
        self.variant_explicit = true;
    }

    /// Perform a hardware reset followed by software reset and initialisation.
    ///
    /// If a drive waveform may still be running (a previous refresh's future
    /// was dropped or errored between master activation and completion —
    /// [`activation_pending`]), wait for BUSY to fall first (bounded by
    /// [`RESET_ACTIVATION_TIMEOUT`]) so the RES# pulse below cannot abort the
    /// waveform mid-phase and freeze the panel (inverted, on SSD1675B).  The
    /// timeout is a backstop against a wedged BUSY; on completion the panel
    /// simply shows the intended frame and the caller's stale shadow/dirty
    /// state self-heals on the next refresh.
    pub async fn reset(&mut self) -> Result<(), I::Error> {
        if self.activation_pending {
            // Only SSD1675B freezes inverted when RES# aborts a waveform
            // mid-phase, so only B pays the wait-out cost.  On A the RES#
            // pulse below aborts the in-flight drive harmlessly, which is
            // exactly what makes the interrupt-driven redraw feel snappy —
            // don't stall it.
            if self.variant == DisplayVariant::Ssd1675B {
                let _ = embassy_time::with_timeout(
                    RESET_ACTIVATION_TIMEOUT,
                    self.interface.busy_wait(),
                )
                .await;
            }
            self.activation_pending = false;
        }
        self.interface.reset().await;
        Command::SoftReset.execute(&mut self.interface).await?;
        self.interface.busy_wait().await?;

        self.init().await
    }

    async fn init(&mut self) -> Result<(), I::Error> {
        Command::AnalogBlockControl(ANALOG_BLOCK_CONTROL_MAGIC)
            .execute(&mut self.interface)
            .await?;
        Command::DigitalBlockControl(DIGITAL_BLOCK_CONTROL_MAGIC)
            .execute(&mut self.interface)
            .await?;

        // Booster soft-start (cmd 0x0C) deliberately omitted — pushing
        // datasheet POR over the OTP-programmed values bricks some panel
        // batches to all-black.  Chip applies its own OTP values on reset.

        // VGH bootstrap (cmd 0x03) — REQUIRED on SSD1675B (POR for VGH is
        // 0x00 / NA per datasheet pg 19, panel won't drive without a valid
        // value).  SSD1675 POR is 0x19 = 21 V, already valid, skip.
        // Sourced from the band-centre OTP entry just to bring the analog
        // rail up during init; every refresh then overwrites VGH with the
        // fixed value from `apply_lut_trailer` (see `VoltageProfile`).
        if self.variant == DisplayVariant::Ssd1675B {
            let vgh = self
                .lut_table
                .map(|t| t[LUT_TABLE_SIZE / 2][100])
                .unwrap_or(0x0E);
            Command::GateDrivingVoltage(vgh)
                .execute(&mut self.interface)
                .await?;
        }

        Command::DriverOutputControl(self.config.dimensions.rows - 1, 0x00)
            .execute(&mut self.interface)
            .await?;

        self.config.dummy_line_period.execute(&mut self.interface).await?;
        self.config.gate_line_width.execute(&mut self.interface).await?;
        if let Some(ref write_vcom) = self.config.write_vcom {
            write_vcom.execute(&mut self.interface).await?;
        }

        self.config.data_entry_mode.execute(&mut self.interface).await?;

        Command::BorderWaveform(0x05).execute(&mut self.interface).await?;

        let end = self.cols_as_bytes() - 1;
        Command::StartEndXPosition(0, end).execute(&mut self.interface).await?;
        Command::StartEndYPosition(0, self.config.dimensions.rows - 1)
            .execute(&mut self.interface)
            .await?;

        Command::XAddress(0x00).execute(&mut self.interface).await?;
        Command::YAddress(0x00).execute(&mut self.interface).await?;

        Ok(())
    }

    /// Full tricolor update using the slow Mode 2 waveform.
    ///
    /// `otp_lut`) into the controller's LUT register before triggering,
    /// since `reset()` callers wipe it every refresh.  Uses 0xEF
    /// (LoadTemp + Mode 2, no LoadLut) so the controller re-samples
    /// on-chip die temperature on every refresh — essential when the
    /// panel can warm up (e.g., direct sunlight) between successive
    /// refreshes.
    ///
    /// Full tri-color refresh.  Picks the LUT entry for the current
    /// [`active_temp_c10`], pushes it via cmd `0x32`, pushes the embedded
    /// voltage trailer, writes both RAM planes, fires a Mode 1
    /// `DISP_CTRL2 = 0xC7` (no `LoadTemp`, no `LoadLut` — chip uses the
    /// pushed LUT verbatim, no TR-search shenanigans).
    ///
    /// # Panics
    /// Caller must register a LUT table via [`register_lut_table`] before
    /// any refresh.  No hardcoded fallback waveform — every panel batch
    /// has its own OTP-tuned WS set that must be probed at boot.
    pub async fn update_tc(&mut self, black: &[u8], red: &[u8], lut_speed: u8) -> Result<(), I::Error> {
        let lut = self.select_full();
        let mut scratch = [0u8; 107];
        let n = scale_lut_into(&mut scratch, &lut[..self.variant.lut_byte_len()], self.variant, lut_speed);
        BufCommand::WriteLUT(&scratch[..n]).execute(&mut self.interface).await?;
        // Trailer pushed from the raw selected LUT (not the scaled scratch)
        // so the voltages stay at OEM levels regardless of `lut_speed`.
        self.apply_lut_trailer(lut).await?;

        self.update_impl(black, red).await?;

        // Mode 1, no `LoadTemp`, no `LoadLut` — pushed LUT used as-is.
        // `0xC7 = EnableClock + EnableAnalog + DisplayMode1 + Display
        //       + DisableAnalog + DisableOsc`.
        Command::UpdateDisplayOption2(
            DisplayUpdateSequenceOption::EnableClockSignal_EnableAnalog_DisplayMode1_DisableAnalog_DisableOscillator,
        )
        .execute(&mut self.interface)
        .await?;
        self.trigger_and_wait().await?;

        Ok(())
    }

    /// B/W refresh.  Same shape as [`update_tc`] but the caller-supplied
    /// `red` plane is typically zeros — only RAM1 (BW) drives visibly.
    /// `mode` selects Mode 1 (`0xC7`) or Mode 2 (`0xCF`) waveform
    /// interpretation; both bypass `LoadTemp` and `LoadLut` so the pushed
    /// LUT is honoured verbatim.
    ///
    /// # Panics
    /// Caller must register a LUT table via [`register_lut_table`] before
    /// any refresh.
    pub async fn update_bw(&mut self, black: &[u8], red: &[u8], mode: UpdateMode, lut_speed: u8) -> Result<(), I::Error> {
        let lut = self.select_no_invert();
        let mut scratch = [0u8; 107];
        let n = scale_lut_into(&mut scratch, &lut[..self.variant.lut_byte_len()], self.variant, lut_speed);
        BufCommand::WriteLUT(&scratch[..n]).execute(&mut self.interface).await?;
        self.apply_lut_trailer(self.otp_band()).await?;

        self.update_impl(black, red).await?;

        let seq = match mode {
            UpdateMode::Mode1 =>
                DisplayUpdateSequenceOption::EnableClockSignal_EnableAnalog_DisplayMode1_DisableAnalog_DisableOscillator,
            UpdateMode::Mode2 =>
                DisplayUpdateSequenceOption::EnableClockSignal_EnableAnalog_DisplayMode2_DisableAnalog_DisableOscillator,
        };
        Command::UpdateDisplayOption2(seq).execute(&mut self.interface).await?;
        self.trigger_and_wait().await?;

        Ok(())
    }

    /// Partial-update refresh driven by the host-side
    /// [`crate::partial::PartialState`] machinery.  Drives only the
    /// pixels marked dirty since the last successful refresh; the
    /// rest see no voltage (routed through LUT3 which the patched
    /// LUT has zeroed).
    ///
    /// `lut` — 107-byte LUT image with the body (bytes 0..70) already
    /// patched for partial mode via [`crate::partial::patch_lut_for_partial`]
    /// and the trailer (70..76) sourced from the OTP-probed entry
    /// for the current temperature band.
    ///
    /// `lut_speed` scales the timing region the same way `update_tc`
    /// + `update_bw` do.
    ///
    /// Returns [`crate::partial::UpdateKind`].  `NoOp` when no pixel
    /// is dirty; `Partial { bbox, had_red }` otherwise.
    pub async fn update_partial(
        &mut self,
        state: &mut crate::partial::PartialState,
        lut_speed: u8,
    ) -> Result<crate::partial::UpdateKind, I::Error> {
        // Recovery: previous update abandoned mid-flight (e.g.
        // `select!` cancellation dropped the future).  Chip may be
        // in a partial state — full reset + re-init recovers it.
        if state.in_flight() {
            self.reset().await?;
            state.set_in_flight(false);
        }

        // Black-edge halo (SSD1675A only): mark the white pixels ringing each
        // dirty black pixel so they re-whiten in the LUT's final phase,
        // erasing the lateral black-ink bleed that fuzzes edges on the
        // overdrive-prone A panel.  Done before bbox so the partial window
        // covers the halo.  Skipped on B (clean, doesn't bleed).
        if self.variant == DisplayVariant::Ssd1675 {
            state.mark_black_halo();
        }

        // Bbox check — no dirty pixels = nothing to do.
        let bbox = match state.bbox_of_dirty() {
            None => return Ok(crate::partial::UpdateKind::NoOp),
            Some(b) => b,
        };

        // Threshold promotion — too many partials since last full,
        // or too much of the panel is dirty.  Run a full refresh
        // instead.  Caller sees `UpdateKind::Full` and knows the
        // panel was driven from scratch.
        if state.should_force_full() {
            self.update_full_from_state(state, lut_speed).await?;
            return Ok(crate::partial::UpdateKind::Full);
        }

        // Build RED + BW planes from pending + dirty.  Returns true
        // if any dirty pixel targets red.
        let had_red = state.build_planes();

        // Base LUT choice:
        //  * No red dirty → start from `no_invert` (TPs and inversion
        //    phases already trimmed at boot by `patch_no_invert`).
        //    Skip the shake-refinement step in `patch_lut_for_partial`
        //    when stacking on this base — the no_invert LUT's
        //    white-wipe phases (LUT0==LUT1==LUT2==0x80 on SSD1675A)
        //    look like shake to the heuristic and get gutted, leaving
        //    black-target pixels mid-gray.  Inversion is already gone
        // Always the non-flashing DELTA LUT — it carries the red drive on
        // LUT2 now, so partial updates (incl. red) never flash.  The flashing
        // LUT is reserved for full refreshes.
        let base_lut = self.select_no_invert();
        let mut lut: [u8; 107] = *base_lut;

        {
            let body_len = self.variant.lut_byte_len();
            let body = &mut lut[..body_len];
            // Delta path (no red dirty) → Preserve invert/wipe phases.
            // The no_invert base's pre-wipe is what gives black-target
            // dirty pixels enough drive energy to reach proper contrast;
            // killing it leaves them gray.
            // Red dirty → Refine OTP shake on the full LUT.
            let invert = if had_red {
                crate::partial::InvertHandling::Refine
            } else {
                // No-red delta path.  On SSD1675 (A) the preserved
                // inversion / pre-wipe phase drives dirty pixels black
                // first, then to target — a visible black flash, and the
                // source of the interrupt re-black artifact (dirty persists
                // across a cancelled refresh → the erase re-runs).  Kill it
                // so A drives straight to target in a single pass, matching
                // the clean SSD1675B behaviour.  B doesn't surface the
                // pre-wipe visibly and its content phases already reach full
                // contrast, so keep Preserve there.
                //
                // NOTE: killing A's pre-wipe also removes the drive energy
                // that pushed black-target pixels to full black.  If black
                // comes out gray after this, boost the LUT0 content-phase
                // drive (on-device contrast tuning).
                match self.variant {
                    DisplayVariant::Ssd1675 => crate::partial::InvertHandling::Kill,
                    DisplayVariant::Ssd1675B => crate::partial::InvertHandling::Preserve,
                }
            };
            crate::partial::patch_lut_for_partial(body, self.variant, invert);
            if !had_red {
                crate::partial::patch_lut_skip_red(body, self.variant);
            }
        }

        // Mark in-flight BEFORE any SPI work.  If cancelled here on,
        // the next call's recovery path will reset the chip.
        state.set_in_flight(true);

        // Push the patched LUT body + apply trailer voltages.  Scale
        // the body's TP region per lut_speed, leave voltage trailer
        // at OEM levels.
        let mut scratch = [0u8; 107];
        let n = scale_lut_into(
            &mut scratch,
            &lut[..self.variant.lut_byte_len()],
            self.variant,
            lut_speed,
        );
        BufCommand::WriteLUT(&scratch[..n])
            .execute(&mut self.interface)
            .await?;
        self.apply_lut_trailer(self.otp_band()).await?;

        // Push RED + BW planes.  Border waveform 0x80 (follow source)
        // matches existing `update_impl` convention.
        let buf_size = self.rows() as usize * self.cols() as usize;
        let buf_limit = (buf_size / 8) + if buf_size % 8 != 0 { 1 } else { 0 };

        self.interface.busy_wait().await?;
        Command::BorderWaveform(0x80)
            .execute(&mut self.interface)
            .await?;

        // RAM2 (Red plane) — cmd 0x26
        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteRedData(&state.red_plane_buf()[..buf_limit])
            .execute(&mut self.interface)
            .await?;

        // RAM1 (BW plane) — cmd 0x24
        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteBlackData(&state.bw_plane_buf()[..buf_limit])
            .execute(&mut self.interface)
            .await?;

        // Fire refresh — Mode 1, no LoadTemp / LoadLut (chip honours
        // the patched LUT verbatim).
        Command::UpdateDisplayOption2(
            DisplayUpdateSequenceOption::EnableClockSignal_EnableAnalog_DisplayMode1_DisableAnalog_DisableOscillator,
        )
        .execute(&mut self.interface)
        .await?;
        self.trigger_and_wait().await?;

        // Count driven pixels into the cumulative since-last-full total
        // (before commit clears the dirty bits) — drives the
        // `full_after_screens` promotion in `should_force_full`.
        let driven = state.dirty_count();

        // Commit — shadow ← snapshot for pixels where pending didn't
        // diverge mid-update.  Dirty bits cleared for those pixels.
        state.commit_refresh();
        state.bump_partial_count();
        state.add_changed_px(driven);
        state.set_in_flight(false);

        Ok(crate::partial::UpdateKind::Partial { bbox, had_red })
    }

    /// Full panel refresh driven from `PartialState`.  Uses the
    /// OTP LUT (with inversion phases — flicker acceptable, DC
    /// balance restored).  Every pixel is encoded via the
    /// `graphics.rs` convention so the chip drives all 4 LUT rows
    /// per the factory tuning.
    ///
    /// Resets `partial_count` on success.  Dirty bits cleared for
    /// pixels where `pending == sent_pending`; pixels that
    /// diverged mid-refresh stay dirty for the next partial.
    pub async fn update_full_from_state(
        &mut self,
        state: &mut crate::partial::PartialState,
        lut_speed: u8,
    ) -> Result<(), I::Error> {
        // Same in-flight recovery semantics as `update_partial`.
        if state.in_flight() {
            self.reset().await?;
            state.set_in_flight(false);
        }

        state.build_planes_full();

        // Full-refresh LUT (with inversion) — the calibrated override when one
        // is installed, else the probed OTP band.  Trailer comes from the same
        // image so its voltages match the waveform.
        let lut = self.select_full();
        let mut scratch = [0u8; 107];
        let n = scale_lut_into(
            &mut scratch,
            &lut[..self.variant.lut_byte_len()],
            self.variant,
            lut_speed,
        );
        BufCommand::WriteLUT(&scratch[..n])
            .execute(&mut self.interface)
            .await?;
        self.apply_lut_trailer(lut).await?;

        let buf_size = self.rows() as usize * self.cols() as usize;
        let buf_limit = (buf_size / 8) + if buf_size % 8 != 0 { 1 } else { 0 };

        // Clear the full-refresh promotion counters NOW, before the
        // cancellable SPI drive.  If this refresh is interrupted (select!
        // cancel — e.g. an animation tick during the multi-second full)
        // before the commit below, the counters must already be cleared;
        // otherwise `should_force_full` stays latched and the next refresh
        // re-triggers a full, repeating indefinitely while interruptions
        // keep arriving (the sleep-sequence repeat bug).
        state.reset_changed_px();
        state.reset_partial_count();

        state.set_in_flight(true);

        self.interface.busy_wait().await?;
        Command::BorderWaveform(0x80)
            .execute(&mut self.interface)
            .await?;

        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteRedData(&state.red_plane_buf()[..buf_limit])
            .execute(&mut self.interface)
            .await?;

        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteBlackData(&state.bw_plane_buf()[..buf_limit])
            .execute(&mut self.interface)
            .await?;

        Command::UpdateDisplayOption2(
            DisplayUpdateSequenceOption::EnableClockSignal_EnableAnalog_DisplayMode1_DisableAnalog_DisableOscillator,
        )
        .execute(&mut self.interface)
        .await?;
        self.trigger_and_wait().await?;

        // Commit shadow ← pending for driven pixels.  Counters were already
        // reset before the drive (interrupt-safe), so don't touch them here.
        state.commit_refresh();
        state.set_in_flight(false);

        Ok(())
    }

    /// Force a full refresh regardless of dirty / threshold state.
    /// Useful at boot, post-tri-color, or as an explicit user
    /// request to clear cumulative ghosting.
    pub async fn force_full_refresh(
        &mut self,
        state: &mut crate::partial::PartialState,
        lut_speed: u8,
    ) -> Result<(), I::Error> {
        // Mark every pixel dirty so commit_refresh inside
        // update_full_from_state captures the full frame into
        // shadow (matches the spec: "drive whole panel.  pixels
        // that moved during full refresh stay dirty").
        state.mark_all_dirty();
        self.update_full_from_state(state, lut_speed).await
    }

    /// Explicit abort + restart.  Issues HW reset, clears in-flight
    /// flag, then runs another `update_partial` so the dirty bitmap
    /// (which the spec says stays sticky across aborts) converges
    /// immediately.  Returns the outcome of the restarted update.
    pub async fn abort_and_restart(
        &mut self,
        state: &mut crate::partial::PartialState,
        lut_speed: u8,
    ) -> Result<crate::partial::UpdateKind, I::Error> {
        self.reset().await?;
        state.set_in_flight(false);
        self.update_partial(state, lut_speed).await
    }

    /// Dispatch the per-variant voltage / timing trailer of an OTP
    /// LUT image to the registers that live OUTSIDE the cmd 0x32
    /// LUT region.  Without this dispatch the controller runs on
    /// power-on-reset voltages — washed-out contrast on every
    /// refresh.  Called after `WriteLUT` in both `update_tc` and
    /// `update_bw` since `init()` resets the controller and we
    /// cannot rely on values surviving a reset cycle.
    ///
    /// Layout (matches the order returned by register 0x33):
    ///
    /// SSD1675 (base = 70):
    ///   +0       VGH        → 0x03 GateDrivingVoltage
    ///   +1..=+3  VSH1/2/VSL → 0x04 SourceDrivingVoltage
    ///   +4       Dummy line → 0x3A DummyLinePeriod
    ///   +5       Gate line  → 0x3B GateLineWidth
    ///   (no VCOM — SSD1675 stores VCOM in a separate OTP region
    ///    that is not part of register 0x33's readback.)
    ///
    /// SSD1675B (base = 100):
    ///   +0       VGH        → 0x03
    ///   +1..=+3  VSH1/2/VSL → 0x04
    ///   +4       VCOM       → 0x2C
    ///   +5       Dummy line → 0x3A
    ///   +6       Gate line  → 0x3B
    /// Dispatch the per-variant voltage / timing trailer of an OTP
    /// LUT image to the registers that live OUTSIDE the cmd 0x32
    /// LUT region.  Without this dispatch the controller runs on
    /// power-on-reset voltages.
    ///
    /// Layout (matches the order returned by register 0x33):
    ///
    /// SSD1675 (base = 70):
    ///   +0       VGH        → 0x03 GateDrivingVoltage
    ///   +1..=+3  VSH1/2/VSL → 0x04 SourceDrivingVoltage
    ///   +4       Dummy line → 0x3A DummyLinePeriod
    ///   +5       Gate line  → 0x3B GateLineWidth
    ///
    /// SSD1675B (base = 100):
    ///   +0       VGH        → 0x03
    ///   +1..=+3  VSH1/2/VSL → 0x04
    ///   +4       VCOM       → 0x2C
    ///   +5       Dummy line → 0x3A
    ///   +6       Gate line  → 0x3B
    /// Push the per-temperature-band drive voltages (cmd
    /// 0x03/0x04/0x2C/0x3A/0x3B), replacing the per-band OTP trailer read.
    /// `src` is the 107-byte LUT image whose waveform was just pushed, so a
    /// calibrated override's voltages ship with its waveform; the partial /
    /// BW paths pass the probed OTP band entry and keep OEM voltages.
    async fn apply_lut_trailer(&mut self, src: &[u8; 107]) -> Result<(), I::Error> {
        let v = match self.variant {
            DisplayVariant::Ssd1675 => {
                // A: 7-byte trailer (VGH,VSH1,VSH2,VSL,dummy,gate at bytes
                // 70..=75) — factory-calibrated voltages AND frame timing for
                // A's panel/clock.  A's OTP has no VCOM byte, so keep the
                // working 0x50.
                VoltageProfile {
                    vgh: src[70],
                    vsh1: src[71],
                    vsh2: src[72],
                    vsl: src[73],
                    vcom: 0x50,
                    dummy: src[74],
                    gate: src[75],
                }
            }
            DisplayVariant::Ssd1675B => {
                // B's voltage trailer lives at bytes 100..=106 (VGH, VSH1,
                // VSH2, VSL, VCOM, dummy, gate).
                VoltageProfile {
                    vgh: src[100],
                    vsh1: src[101],
                    vsh2: src[102],
                    vsl: src[103],
                    vcom: src[104],
                    dummy: src[105],
                    gate: src[106],
                }
            }
        };
        Command::GateDrivingVoltage(v.vgh)
            .execute(&mut self.interface)
            .await?;
        Command::SourceDrivingVoltage(v.vsh1, v.vsh2, v.vsl)
            .execute(&mut self.interface)
            .await?;
        Command::WriteVCOM(v.vcom)
            .execute(&mut self.interface)
            .await?;
        Command::DummyLinePeriod(v.dummy)
            .execute(&mut self.interface)
            .await?;
        Command::GateLineWidth(v.gate)
            .execute(&mut self.interface)
            .await?;
        Ok(())
    }

    /// Common back-end for `update_tc` / `update_bw` (full-refresh
    /// paths).  Sets the border-waveform register, then writes the
    /// caller-supplied black plane to BW RAM (cmd 0x24) and the
    /// caller-supplied second plane to red RAM (cmd 0x26).
    ///
    /// `border` is the value pushed to register 0x3C.  Full-refresh
    /// callers pass `0x05` (matches the value used in `init`).  The
    /// previous unconditional `0x80` was a partial-update style
    /// border that left the wrong setting active for full refreshes.
    ///
    /// `red_or_mirror`: the buffer written to cmd 0x26 (red RAM).
    /// - For tri-color full refresh (`update_tc`), the caller passes
    ///   the user's red plane.
    /// - For B/W full refresh (`update_bw`), the caller passes the
    ///   `black` slice itself so RAM 0x26 mirrors the displayed image
    ///   — required for correct partial-update baselines on the next
    ///   partial refresh.  Without this mirror, RAM 0x26 holds zeros
    ///   (or stale red) and the partial diff ghosts.
    /// B/W-only variant of `update_impl` (O1).  Sets the red-RAM
    async fn update_impl(&mut self, black: &[u8], red: &[u8]) -> Result<(), I::Error> {
        self.interface.busy_wait().await?;

        Command::BorderWaveform(0x80).execute(&mut self.interface).await?;

        let buf_size = self.rows() as usize * self.cols() as usize;
        let limit_adder = if buf_size % 8 != 0 { 1 } else { 0 };
        let buf_limit = (buf_size / 8) + limit_adder;

        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteBlackData(&black[..buf_limit]).execute(&mut self.interface).await?;

        Command::XAddress(0).execute(&mut self.interface).await?;
        Command::YAddress(0).execute(&mut self.interface).await?;
        BufCommand::WriteRedData(&red[..buf_limit]).execute(&mut self.interface).await?;

        Ok(())
    }

    pub async fn partial_update(
        &mut self,
        image: &[u8],
        start_x_px: u16,
        start_y_px: u16,
        width_px: u16,
        height_px: u16,
    ) -> Result<(), I::Error> {
        self.interface.busy_wait().await?;

        Command::BorderWaveform(0x80).execute(&mut self.interface).await?;

        let start_x_byte = (start_x_px / 8) as u8;
        let width_byte = (width_px / 8) as u8;
        let end_x_byte = start_x_byte + width_byte - 1;
        Command::StartEndXPosition(start_x_byte, end_x_byte)
            .execute(&mut self.interface)
            .await?;
        Command::StartEndYPosition(start_y_px, start_y_px + height_px - 1)
            .execute(&mut self.interface)
            .await?;

        Command::XAddress(start_x_byte).execute(&mut self.interface).await?;
        Command::YAddress(start_y_px).execute(&mut self.interface).await?;

        BufCommand::WriteBlackData(image).execute(&mut self.interface).await?;

        Command::UpdateDisplayOption2(
            DisplayUpdateSequenceOption::EnableClockSignal_LoadTemp_EnableAnalog_DisplayMode1_DisableAnalog_DisableOscillator,
        )
        .execute(&mut self.interface)
        .await?;
        self.trigger_and_wait().await?;

        Command::StartEndXPosition(0, self.cols_as_bytes() - 1)
            .execute(&mut self.interface)
            .await?;
        Command::StartEndYPosition(0, self.config.dimensions.rows - 1)
            .execute(&mut self.interface)
            .await?;

        Ok(())
    }

    /// True if this is the SSD1675B (10-phase) controller variant.
    #[cfg(feature = "staged")]
    pub fn is_b_variant(&self) -> bool {
        matches!(self.variant, DisplayVariant::Ssd1675B)
    }

    /// Upload a short staged LUT body (cmd 0x32) then push the temperature
    /// voltage trailer.  Body holds per-phase selectors; trailer supplies magnitudes.
    ///
    /// # Arguments
    ///
    /// * `lut` - the stage LUT to encode and upload
    ///
    /// # Errors
    ///
    /// Propagates any `I::Error` from the underlying bus writes.
    #[cfg(feature = "staged")]
    pub async fn staged_upload_lut(
        &mut self,
        lut: &crate::staged::StageLut,
    ) -> Result<(), I::Error> {
        let mut body = [0u8; crate::staged::MAX_BODY];
        let len = lut.encode(self.is_b_variant(), &mut body);
        self.interface.send_command(0x32).await?;
        self.interface.send_data(&body[..len]).await?;
        self.apply_lut_trailer(self.otp_band()).await
    }

    /// Window + write both RAM planes for one stage.
    ///
    /// # Arguments
    ///
    /// * `bw_plane` - the windowed black/white plane bytes
    /// * `red_plane` - the windowed red plane bytes
    /// * `region` - the RAM window to address
    ///
    /// # Errors
    ///
    /// Propagates any `I::Error` from the underlying bus writes.
    #[cfg(feature = "staged")]
    pub async fn staged_upload_planes(
        &mut self,
        bw_plane: &[u8],
        red_plane: &[u8],
        region: crate::staged::Region,
    ) -> Result<(), I::Error> {
        crate::staged::upload_planes(&mut self.interface, bw_plane, red_plane, region).await
    }

    /// Trigger one stage activation: BorderWaveform → Mode1 Option2 → UpdateDisplay → busy wait.
    ///
    /// # Errors
    ///
    /// Propagates any `I::Error` from the underlying bus writes or busy wait.
    #[cfg(feature = "staged")]
    pub async fn staged_trigger(&mut self) -> Result<(), I::Error> {
        crate::staged::trigger_stage(&mut self.interface).await
    }

    pub async fn deep_sleep(&mut self) -> Result<(), I::Error> {
        Command::DeepSleepMode(DeepSleepMode::PreserveRAM)
            .execute(&mut self.interface)
            .await
    }

    pub fn rows(&self) -> u16 {
        self.config.dimensions.rows
    }

    pub fn cols(&self) -> u8 {
        self.config.dimensions.cols
    }

    pub fn cols_as_bytes(&self) -> u8 {
        self.config.dimensions.cols / 8
    }

    pub fn rotation(&self) -> Rotation {
        self.config.rotation
    }
}

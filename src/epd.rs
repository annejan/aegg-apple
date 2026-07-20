//! E-paper panel bring-up (SSD1675 / SSD1675B over SPI3).
//!
//! Ported from the CyberAegg badge firmware (`bornhack-firmware-2026`,
//! `src/fw/epd.rs`), stripped down to what a standalone video player needs.
//!
//! ## What this module does
//!
//! [`Panel::init`] brings up SPI3 + the six panel GPIOs, reads the
//! controller's OTP waveform back out of the LUT register (see
//! [`probe_lut`]), auto-detects the controller variant from that image, and
//! registers both the raw and the "no-invert" LUT tables with the driver.
//! [`Panel::show`] then pushes one 1bpp plane per video frame through the
//! flicker-free fast waveform.
//!
//! ## Differences from the badge firmware (deliberate)
//!
//! * **One OTP probe, not sixteen.**  The badge synthesises 16 temperature
//!   values, probes the OTP once per value and builds a 16-band
//!   temperature-compensated table (~2-3 s of boot time).  This firmware
//!   plays a 3.5-minute video indoors at room temperature; the panel does
//!   not move between bands during a single playthrough, so the
//!   compensation buys nothing and the boot cost is pure latency before the
//!   video starts.  We probe **once** (letting the controller pick the band
//!   from its own internal sensor) and replicate that single 107-byte image
//!   into all 16 entries of both tables, so the driver's `band_idx()`
//!   selects the same waveform whatever temperature it thinks it is at.
//! * **No `LUT.CFG` parsing, no KV persistence, no temperature bias, no
//!   Fire-held-at-boot OTP recovery, no baked-in per-variant calibration
//!   overrides.**  There is no filesystem config, no settings UI and no
//!   menu to recover into on this build.
//! * **No partial-refresh machinery.**  Every video frame changes most of
//!   the panel, so the host-side dirty tracking, shadow buffers and ~46 KB
//!   of `.bss` that `ssd1675::partial` needs would buy nothing.
//!   [`Panel::show`] drives `Display::update_bw` directly.

use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin as GpioPin, Port, Pull};
use embassy_nrf::spim::{Config, Frequency, InterruptHandler, Spim};
use embassy_nrf::{Peri, bind_interrupts, peripherals};
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use ssd1675::{
    Builder, Color, Dimensions, Display, DisplayVariant, GraphicDisplay, Interface,
    LUT_TABLE_SIZE, Rotation, UpdateMode, detect_variant_from_otp, patch_no_invert,
};
use static_cell::StaticCell;

use core::sync::atomic::{AtomicBool, Ordering};

/// Panel geometry: 152 × 152 pixels.
const ROWS: u16 = 152;
const COLS: u8 = 152;

/// One 1bpp plane, `152 * 152 / 8` bytes.
pub const BUF_SIZE: usize = ROWS as usize * COLS as usize / 8;

bind_interrupts!(struct Irqs {
    SPIM3 => InterruptHandler<peripherals::SPI3>;
});

type EpdGfx = GraphicDisplay<
    'static,
    Interface<
        ExclusiveDevice<Spim<'static>, Output<'static>, embassy_time::Delay>,
        Input<'static>,
        Output<'static>,
        Output<'static>,
    >,
    &'static mut [u8],
>;

/// Boot-probed LUT table — the full OTP waveform, inversion phases intact.
/// 16 × 107 = 1.7 KB.  Used by `update_tc` for the flashing full refresh
/// that resets ghosting.  All 16 bands hold the same image (see the module
/// docs on why temperature compensation is dropped here).
static LUT_TABLE_CELL: StaticCell<[[u8; 107]; LUT_TABLE_SIZE]> = StaticCell::new();
/// Same as [`LUT_TABLE_CELL`] but with the inversion phases zeroed per
/// `patch_no_invert`.  Used by `update_bw` for the flicker-free fast
/// refresh that every video frame goes through.
static LUT_TABLE_NO_INVERT_CELL: StaticCell<[[u8; 107]; LUT_TABLE_SIZE]> = StaticCell::new();

/// The three planes `GraphicDisplay` insists on owning.  Only
/// [`Panel::clear_white`] actually draws through them; [`Panel::show`]
/// hands the caller's frame straight to the driver.
static BLACK_CELL: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
static RED_CELL: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();
static WORK_CELL: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();

/// The red plane handed to `update_bw` on every frame.  This panel is
/// driven black/white only, so it is permanently zero.
static RED_ZEROS: [u8; BUF_SIZE] = [0u8; BUF_SIZE];

/// Single-shot guard: [`Panel::init`] steals SPI3 and the GPIOs during the
/// OTP probe, so calling it twice would hand out two owners of the same
/// peripherals.
static INIT_DONE: AtomicBool = AtomicBool::new(false);

fn pin_nr(p: &Peri<'_, AnyPin>) -> u8 {
    let port = match p.port() {
        Port::Port0 => 0u8,
        Port::Port1 => 1u8,
    };
    port * 32 + p.pin()
}

/// Read back the OTP LUT register (command 0x33) using stolen peripherals.
///
/// Sequence (per SSD1619 reference driver):
///   1. Hardware reset + 100 ms settle
///   2. Select the on-chip internal temperature sensor (0x18 = 0x80) — the
///      SSD1675 will use its own die measurement when the next LoadTemp step
///      runs.  The SoC's idea of temperature is *not* written: the panel's
///      internal sensor is more representative of the panel itself than the
///      nRF52840's die.
///   3. Send 0x22 / 0xB1 — EnableClock | LoadTemp | LoadLUT-Mode1 |
///      DisableClock
///   4. Send 0x20 — Master Activation (BUSY goes HIGH while controller loads
///      OTP zone)
///   5. Wait for BUSY LOW (controller has finished loading the temperature
///      zone into the LUT register)
///   6. Send 0x33 command then read 107 bytes — the loaded LUT zone
///
/// The badge firmware instead writes a synthesised temperature via cmd 0x1A
/// and uses `0x22 = 0x91` (no LoadTemp) so the chip's TR-search lands in a
/// different band on each of its 16 passes.  Here we probe exactly once and
/// *want* the controller's own idea of the current band, so the documented
/// LoadTemp form above is used verbatim and the 0x1A write is gone.
///
/// All stolen resources are dropped (or `mem::forget`'d, see below) before
/// returning.
async fn probe_lut(
    sck: &Peri<'_, AnyPin>,
    data: &Peri<'_, AnyPin>,
    cs: &Peri<'_, AnyPin>,
    dc: &Peri<'_, AnyPin>,
    rst: &Peri<'_, AnyPin>,
    busy: &Peri<'_, AnyPin>,
) -> [u8; 107] {
    let sck_nr = pin_nr(sck);
    let data_nr = pin_nr(data);
    let cs_nr = pin_nr(cs);
    let dc_nr = pin_nr(dc);
    let rst_nr = pin_nr(rst);
    let busy_nr = pin_nr(busy);

    // GPIO wrappers are mem::forget'd at the end to preserve pin config.
    let mut cs_out = Output::new(
        unsafe { AnyPin::steal(cs_nr) },
        Level::High,
        OutputDrive::Standard,
    );
    let mut dc_out = Output::new(
        unsafe { AnyPin::steal(dc_nr) },
        Level::Low,
        OutputDrive::Standard,
    );
    let mut rst_out = Output::new(
        unsafe { AnyPin::steal(rst_nr) },
        Level::Low,
        OutputDrive::Standard,
    );
    let busy_in = Input::new(unsafe { AnyPin::steal(busy_nr) }, Pull::Down);

    let mut cfg = Config::default();
    // SSD1675 datasheet rates SCK up to ~20 MHz; SPIM3 caps at 32 MHz.
    // 16 MHz is comfortably below both.
    cfg.frequency = Frequency::M16;

    // Hardware reset — flat 100 ms settle (BUSY does not reliably pulse during
    // reset/OTP boot).
    Timer::after_millis(10).await;
    rst_out.set_high();
    Timer::after_millis(100).await;

    // Phase 0: SoftReset + analog/digital block setup.  Matches the
    // badge.team SSD168x init pattern (HW reset → 0x12 → 0x74 → 0x7E
    // → ...).  Without these, the OTP zone reload doesn't execute and the
    // LUT readback comes back as whatever the register happened to hold.
    cs_out.set_low();
    {
        let mut spi_tx = Spim::new_txonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg.clone(),
        );
        // 0x12 = SoftReset.  Puts chip in known state; BUSY pulses
        // high then low while internal logic clears.
        dc_out.set_low();
        spi_tx.write(&[0x12]).await.ok();
        dc_out.set_high();
        core::mem::forget(spi_tx);
    }
    cs_out.set_high();
    for _ in 0..100u8 {
        if !busy_in.is_high() {
            break;
        }
        Timer::after_millis(10).await;
    }

    cs_out.set_low();
    {
        let mut spi_tx = Spim::new_txonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg.clone(),
        );
        // 0x74 = AnalogBlockControl (value 0x54 per datasheet).
        dc_out.set_low();
        spi_tx.write(&[0x74]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0x54]).await.ok();
        // 0x7E = DigitalBlockControl (value 0x3B per datasheet).
        dc_out.set_low();
        spi_tx.write(&[0x7E]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0x3B]).await.ok();
        core::mem::forget(spi_tx);
    }
    cs_out.set_high();

    // Phase 1: select the internal temperature sensor and trigger the OTP
    // LUT zone load.
    cs_out.set_low();
    {
        let mut spi_tx = Spim::new_txonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg.clone(),
        );
        // 0x18 = 0x80: select the *internal* temperature sensor (B-variant
        // documented; A-variant accepts as a no-op per the gap on pg 23).
        // The LoadTemp step below then samples it, so the controller's own
        // TR-search picks the band for the actual ambient temperature.
        dc_out.set_low();
        spi_tx.write(&[0x18]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0x80]).await.ok();
        // 0x22 / 0xB1: EnableClock | LoadTemp | LoadLUT-OTP-Mode1 |
        // DisableClock.  LoadTemp re-samples the sensor selected by 0x18 —
        // exactly what we want for a single probe.
        dc_out.set_low();
        spi_tx.write(&[0x22]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0xB1]).await.ok();
        // 0x20: Master Activation — BUSY goes HIGH while the controller loads
        // the OTP zone.
        dc_out.set_low();
        spi_tx.write(&[0x20]).await.ok();
        // Don't drop — Spim::drop disconnects SPI pins.
        core::mem::forget(spi_tx);
    }
    cs_out.set_high();

    // Wait for BUSY LOW: controller has finished loading the temperature zone into
    // the LUT register. Poll every 10 ms, up to 1 s total.
    for _ in 0..100u8 {
        if !busy_in.is_high() {
            break;
        }
        Timer::after_millis(10).await;
    }

    // Phase 2: read 107 bytes from the LUT register (0x33).
    // The controller now presents the loaded zone on MISO.
    // Stack-allocated only for the duration of the SPI read; caller moves it into
    // StaticCell.
    let mut lut = [0u8; 107];
    cs_out.set_low();
    {
        // Command phase: send 0x33 on MOSI.
        let mut spi_tx = Spim::new_txonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg.clone(),
        );
        dc_out.set_low();
        spi_tx.write(&[0x33]).await.ok();
        dc_out.set_high();
        core::mem::forget(spi_tx);
    }
    {
        // Data phase: read 107 bytes on MISO (same physical pin, now input).
        let mut spi_rx = Spim::new_rxonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg.clone(),
        );
        spi_rx.read(&mut lut).await.ok();
        // Drop the RX Spim — it will disable SPI3, but we restore TX mode below.
        drop(spi_rx);
    }
    cs_out.set_high();

    // Restore SPI3 to TX-only mode (data pin as MOSI) so the display's
    // Spim can transmit. The display's Spim doesn't reconfigure pin
    // selection on each write — it was set once at boot.
    {
        let restore = Spim::new_txonly(
            unsafe { peripherals::SPI3::steal() },
            Irqs,
            unsafe { AnyPin::steal(sck_nr) },
            unsafe { AnyPin::steal(data_nr) },
            cfg,
        );
        core::mem::forget(restore);
    }

    defmt::debug!("Display OTP LUT (107 bytes):");
    for (i, chunk) in lut.chunks(10).enumerate() {
        defmt::debug!("  [{=usize:03}] {:02x}", i * 10, chunk);
    }

    // Prevent Drop from disconnecting GPIO pins — the display's real
    // Output/Input instances still own these pins.
    core::mem::forget(cs_out);
    core::mem::forget(dc_out);
    core::mem::forget(rst_out);
    core::mem::forget(busy_in);

    lut
}

/// The e-paper panel, ready to be fed video frames.
pub struct Panel {
    gfx: EpdGfx,
    variant: DisplayVariant,
    /// Waveform frames in the fast (inversion-stripped) LUT. Zero means it
    /// drives nothing and `show` must use the full waveform instead.
    fast_frames: u32,
}

impl Panel {
    /// Bring up SPI3 + pins, probe the panel's OTP waveform, register LUTs.
    ///
    /// May only be called once — the OTP probe steals SPI3 and all six pins
    /// out from under the `Peri` handles, so a second caller would end up
    /// sharing them.
    pub async fn init(
        spi3: Peri<'static, peripherals::SPI3>,
        sck: Peri<'static, AnyPin>,
        mosi: Peri<'static, AnyPin>,
        cs: Peri<'static, AnyPin>,
        dc: Peri<'static, AnyPin>,
        rst: Peri<'static, AnyPin>,
        busy: Peri<'static, AnyPin>,
    ) -> Panel {
        if INIT_DONE.swap(true, Ordering::Relaxed) {
            defmt::panic!("Panel::init called twice");
        }

        // Allocate the tables in static storage first, then fill in-place —
        // keeps the 2 × 1.7 KB arrays off the (small) boot stack.
        let lut_table: &'static mut [[u8; 107]; LUT_TABLE_SIZE] =
            LUT_TABLE_CELL.init([[0u8; 107]; LUT_TABLE_SIZE]);
        let lut_table_no_invert: &'static mut [[u8; 107]; LUT_TABLE_SIZE] =
            LUT_TABLE_NO_INVERT_CELL.init([[0u8; 107]; LUT_TABLE_SIZE]);

        // Single OTP probe.  The badge firmware runs this 16 times against
        // synthesised temperatures to build a temperature-compensated band
        // table, at 2-3 s of boot cost; a room-temperature video player does
        // not need that, so the one probed image is replicated into every
        // band and `Display::band_idx()` becomes a no-op selector.
        let otp = probe_lut(&sck, &mosi, &cs, &dc, &rst, &busy).await;
        for band in lut_table.iter_mut() {
            *band = otp;
        }

        // Detect the controller variant from the probed image — decides the
        // LUT row/timing layout, which `patch_no_invert` and the driver's
        // scaling both depend on.
        let variant = detect_variant_from_otp(&otp);
        defmt::info!(
            "EPD controller: {}",
            match variant {
                DisplayVariant::Ssd1675B => "SSD1675B (10-byte row LUT)",
                DisplayVariant::Ssd1675 => "SSD1675 (7-byte row LUT)",
            }
        );

        // Derive the flicker-free table: same waveform with the inversion /
        // pre-charge phases zeroed.  Replicated into every band for the same
        // reason as above.
        let mut no_invert = otp;
        patch_no_invert(&mut no_invert, variant);
        for band in lut_table_no_invert.iter_mut() {
            *band = no_invert;
        }

        // Build the SPI bus.  Same M16 as the probe path above: refresh time
        // is waveform-bound (LUT timings, not SCK), but a faster bus frees
        // the executor sooner during the plane push so the audio task keeps
        // its deadlines.
        let mut cfg = Config::default();
        cfg.frequency = Frequency::M16;
        let bus = Spim::new_txonly(spi3, Irqs, sck, mosi, cfg);

        let cs_out = Output::new(cs, Level::High, OutputDrive::Standard);
        let rst_out = Output::new(rst, Level::Low, OutputDrive::Standard);
        let dc_out = Output::new(dc, Level::Low, OutputDrive::Standard);
        let busy_in = Input::new(busy, Pull::Down);

        let spi_dev = ExclusiveDevice::new(bus, cs_out, embassy_time::Delay).unwrap();
        let controller = Interface::new(spi_dev, busy_in, dc_out, rst_out);
        let config = Builder::new()
            .dimensions(Dimensions {
                rows: ROWS,
                cols: COLS,
            })
            .rotation(Rotation::Rotate0)
            .build()
            .unwrap();

        let mut gfx = GraphicDisplay::new(
            Display::new(controller, config),
            BLACK_CELL.init([0u8; BUF_SIZE]).as_mut_slice(),
            RED_CELL.init([0u8; BUF_SIZE]).as_mut_slice(),
            WORK_CELL.init([0u8; BUF_SIZE]).as_mut_slice(),
        );
        // Must precede any refresh: `reset()` is variant-gated (the VGH
        // bootstrap, cmd 0x03, is only sent on B).
        gfx.set_variant(variant);

        // `update_bw` / `update_tc` panic unless both tables are registered,
        // so this has to happen before anything drives the panel.
        gfx.register_lut_tables(lut_table, lut_table_no_invert);

        // Hardware + soft reset and full controller init (driver output
        // control, data entry mode, RAM window).  The probe above left the
        // chip only partly configured, and `reset()` needs the LUT table
        // registered on B for the VGH bootstrap — hence this ordering.
        if gfx.reset().await.is_err() {
            defmt::error!("EPD reset/init failed");
        }

        // How much drive the fast waveform actually has.
        //
        // `patch_no_invert` strips the inversion / pre-charge phases out of
        // the probed OTP waveform. On some panels those are most of the
        // waveform, and what is left drives the pixels barely or not at all --
        // a refresh that changes nothing and never even raises BUSY. Measure
        // it once here so `show` can fall back rather than silently drawing
        // blank frames for three and a half minutes.
        let fast_frames = ssd1675::waveform_frames(&lut_table_no_invert[0], variant);
        let full_frames = ssd1675::waveform_frames(&lut_table[0], variant);
        defmt::info!(
            "EPD waveform frames: fast {}, full {}",
            fast_frames,
            full_frames
        );
        if fast_frames == 0 {
            defmt::warn!("fast waveform is empty; falling back to the full waveform");
        }

        Panel {
            gfx,
            variant,
            fast_frames,
        }
    }

    /// Push one 1bpp black plane and drive a fast non-flashing refresh.
    /// `speed` is the LUT timing scale, 30 (fastest) ..= 255 — 100 is the
    /// OEM duration, lower stretches less time per waveform phase.
    ///
    /// Deliberately does **not** go through `ssd1675::partial`: a video frame
    /// changes most of the panel, so host-side dirty tracking would find
    /// nothing to skip while costing ~46 KB of `.bss`.  `update_bw` picks
    /// the `select_no_invert()` waveform — the flicker-free one.
    pub async fn show(&mut self, black: &[u8; BUF_SIZE], speed: u8) {
        // When the fast waveform is empty it cannot draw anything, so use the
        // full one. It flashes and is far slower, but a slow visible video
        // beats a fast invisible one.
        if self.fast_frames == 0 {
            self.gfx.black_buffer_mut().copy_from_slice(black);
            if self.gfx.update_tc(speed).await.is_err() {
                defmt::error!("EPD full-waveform frame refresh failed");
            }
            return;
        }

        let display: &mut Display<'static, _> = &mut self.gfx;
        if display
            .update_bw(black, &RED_ZEROS, UpdateMode::Mode1, speed)
            .await
            .is_err()
        {
            defmt::error!("EPD frame refresh failed");
        }
    }

    /// Full flashing tri-colour refresh to clean white.  Use at start/end:
    /// the inversion phases in the raw OTP waveform are what actually clear
    /// accumulated ghosting from the non-flashing frame path.
    pub async fn clear_white(&mut self) {
        self.gfx.clear(Color::White);
        if self.gfx.reset().await.is_err() {
            defmt::error!("EPD reset before clear failed");
        }
        // OEM timing (the variant default) — a full refresh is a one-off at
        // start/end, so there is nothing to gain by rushing it.
        let speed = self.variant.default_lut_speed();
        if self.gfx.update_tc(speed).await.is_err() {
            defmt::error!("EPD clear-to-white failed");
        }
    }

    /// Put the controller into deep sleep (RAM preserved).  Any later
    /// refresh must be preceded by a reset, so treat this as terminal for
    /// the playback session.
    pub async fn sleep(&mut self) {
        let display: &mut Display<'static, _> = &mut self.gfx;
        if display.deep_sleep().await.is_err() {
            defmt::error!("EPD deep sleep failed");
        }
    }
}

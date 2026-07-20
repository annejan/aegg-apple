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
use ssd1675::partial::{PartialConfig, PartialState, sync_from_planes};
use ssd1675::{
    Builder, Color, Dimensions, Display, DisplayVariant, GraphicDisplay, Interface,
    LUT_TABLE_SIZE, Rotation, detect_variant_from_otp, patch_no_invert,
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
///
/// It must live in **RAM**, not `.rodata`.  A plain `static [u8; N]` is
/// read-only data in flash, and EasyDMA cannot read flash: `embassy-nrf`
/// quietly falls back to copying the transfer through a 512-byte RAM bounce
/// buffer, which panics on anything larger — and a plane is 2888 bytes.  That
/// is exactly why `clear_white` worked (it uses the driver's own
/// `StaticCell` planes, already in RAM) while every `show` died.
static RED_ZEROS_CELL: StaticCell<[u8; BUF_SIZE]> = StaticCell::new();

/// Shadow/dirty state for the partial-refresh engine (~46 KB of .bss).
///
/// Worth it: `update_partial` is the path the badge firmware actually drives
/// the panel with, at roughly 500 ms a refresh. Feeding the no-invert LUT
/// straight to `update_bw` instead -- a path nothing else exercises -- leaves
/// the controller driving forever with BUSY never falling.
static PARTIAL_CELL: StaticCell<PartialState> = StaticCell::new();

/// Temperature handed to the OTP TR-search, 12-bit signed in 1/16 °C.
/// 0x190 = 400 = 25 °C, which is also the controller's own POR default.
const ROOM_TEMP_RAW: u16 = 25 * 16;

/// Single-shot guard: [`Panel::init`] steals SPI3 and the GPIOs during the
/// OTP probe, so calling it twice would hand out two owners of the same
/// peripherals.
static INIT_DONE: AtomicBool = AtomicBool::new(false);

/// Read the BUSY pin straight out of the GPIO peripheral.
///
/// The driver owns the `Input`, so this is the only way to observe the line
/// from outside it. Worth having: the panel's BUSY behaviour is the thing
/// that decides whether a refresh ever completes, and a stuck or inverted
/// BUSY looks exactly like a hung player from the outside.
pub fn busy_level() -> bool {
    // P0_14.
    (embassy_nrf::pac::P0.in_().read().0 >> 14) & 1 != 0
}

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
///   2. Select the temperature sensor (0x18 = 0x80), then write the
///      temperature manually via 0x1A — the controller has no on-die sensor
///      to sample.
///   3. Send 0x22 / 0x91 — EnableClock | LoadLUT-Mode1 | DisableClock, with
///      no LoadTemp bit so the written temperature survives
///   4. Send 0x20 — Master Activation (BUSY goes HIGH while controller loads
///      OTP zone)
///   5. Wait for BUSY LOW (controller has finished loading the temperature
///      zone into the LUT register)
///   6. Send 0x33 command then read 107 bytes — the loaded LUT zone
///
/// `temp_raw` is the temperature the TR-search matches against, 12-bit
/// signed in 1/16 °C.  The badge firmware sweeps 16 synthesised values to
/// build a per-band table; this firmware passes one room-temperature value,
/// because a single indoor playthrough never leaves the band.
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
    temp_raw: u16,
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
        // 0x18 = 0x80: select the internal temperature sensor.
        dc_out.set_low();
        spi_tx.write(&[0x18]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0x80]).await.ok();
        // 0x1A: write the temperature manually, 12-bit signed in 1/16 °C.
        //
        // This must not be skipped. The SSD1675 has no on-die temperature
        // sensor (datasheet pg 6 block diagram) and this badge wires no
        // external one, so there is nothing for a LoadTemp step to sample:
        // the register would sit at its POR value and the §6.9 TR-search
        // would match a waveform that does not drive the panel. An earlier
        // version of this probe used LoadTemp and no 0x1A write, and the
        // resulting LUT measured 1010 waveform frames against the 1345 the
        // panel's OTP actually holds -- every refresh then ran forever with
        // BUSY never falling.
        let byte1 = ((temp_raw >> 4) & 0xFF) as u8;
        let byte2 = ((temp_raw & 0x0F) << 4) as u8;
        dc_out.set_low();
        spi_tx.write(&[0x1A]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[byte1, byte2]).await.ok();
        // 0x22 / 0x91: EnableClock | LoadLUT-OTP-Mode1 | DisableClock.
        // Deliberately NOT 0xB1: the LoadTemp bit would re-sample the
        // absent sensor and clobber the value just written.
        dc_out.set_low();
        spi_tx.write(&[0x22]).await.ok();
        dc_out.set_high();
        spi_tx.write(&[0x91]).await.ok();
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
    /// Permanently-zero red plane, in RAM so EasyDMA can read it.
    red_zeros: &'static [u8; BUF_SIZE],
    /// Host-side shadow + dirty tracking for the partial-refresh engine.
    partial: &'static mut PartialState,
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
        let otp = probe_lut(&sck, &mosi, &cs, &dc, &rst, &busy, ROOM_TEMP_RAW).await;
        for band in lut_table.iter_mut() {
            *band = otp;
        }

        // Detect the controller variant from the probed image — decides the
        // LUT row/timing layout, which `patch_no_invert` and the driver's
        // scaling both depend on.
        let variant = detect_variant_from_otp(&otp);
        let variant_name = match variant {
            DisplayVariant::Ssd1675B => "SSD1675B (10-byte row LUT)",
            DisplayVariant::Ssd1675 => "SSD1675 (7-byte row LUT)",
        };
        defmt::info!("EPD controller: {}", variant_name);
        crate::log!("EPD controller: {}", variant_name);

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
            crate::log!("EPD reset/init failed");
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
        crate::log!(
            "EPD waveform frames: fast {}, full {}",
            fast_frames,
            full_frames
        );
        if fast_frames == 0 {
            defmt::warn!("fast waveform is empty; falling back to the full waveform");
            crate::log!("fast waveform is empty; falling back to the full waveform");
        }

        Panel {
            gfx,
            variant,
            red_zeros: RED_ZEROS_CELL.init([0u8; BUF_SIZE]),
            partial: {
                let state = PARTIAL_CELL.init(PartialState::take(ROWS, COLS as u16));
                // Never promote to a full refresh. The stock firmware does so
                // every few screens to clear ghosting, but here the ghosting
                // is wanted and a full waveform costs ~12 s -- fifty frames.
                state.set_config(PartialConfig {
                    full_after_screens: u32::MAX,
                    ..PartialConfig::default()
                });
                state
            },
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
        // No per-frame reset. It was added while chasing what turned out to
        // be the DMA panic, and it is actively harmful: `reset()` waits for
        // BUSY to fall, so on a panel still busy from the previous frame it
        // eats the entire refresh budget before any pixels are driven.
        // `update_bw` re-pushes the LUT and the RAM pointers itself.

        // When the fast waveform is empty it cannot draw anything, so use the
        // full one. It flashes and is far slower, but a slow visible video
        // beats a fast invisible one.
        if self.fast_frames == 0 {
            self.gfx.black_buffer_mut().copy_from_slice(black);
            if self.gfx.update_tc(speed).await.is_err() {
                defmt::error!("EPD full-waveform frame refresh failed");
                crate::log!("EPD full-waveform frame refresh failed");
            }
            return;
        }

        let t0 = embassy_time::Instant::now();

        // Drive through the partial-refresh engine, the same path the badge
        // firmware uses. It diffs against a host-side shadow and drives only
        // the pixels that changed, with the inversion phases patched out, so
        // there is no flash and no full-screen erase -- the ghosting just
        // accumulates, which for this is the point.
        self.gfx.black_buffer_mut().copy_from_slice(black);
        self.gfx.red_buffer_mut().copy_from_slice(self.red_zeros);
        sync_from_planes(self.partial, black, self.red_zeros);

        let display: &mut Display<'static, _> = &mut self.gfx;
        match display.update_partial(self.partial, speed).await {
            Ok(kind) => defmt::info!(
                "partial {} ms, kind {}, busy {}",
                t0.elapsed().as_millis(),
                defmt::Debug2Format(&kind),
                busy_level() as u8
            ),
            Err(_) => {
                defmt::error!("EPD partial refresh failed");
                crate::log!("EPD partial refresh failed");
            }
        }
    }

    /// Full flashing tri-colour refresh to clean white.  Use at start/end:
    /// the inversion phases in the raw OTP waveform are what actually clear
    /// accumulated ghosting from the non-flashing frame path.
    pub async fn clear_white(&mut self) {
        self.gfx.clear(Color::White);
        if self.gfx.reset().await.is_err() {
            defmt::error!("EPD reset before clear failed");
            crate::log!("EPD reset before clear failed");
        }
        // OEM timing (the variant default) — a full refresh is a one-off at
        // start/end, so there is nothing to gain by rushing it.
        let speed = self.variant.default_lut_speed();
        if self.gfx.update_tc(speed).await.is_err() {
            defmt::error!("EPD clear-to-white failed");
            crate::log!("EPD clear-to-white failed");
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

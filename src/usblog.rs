//! USB CDC-ACM logging -- a serial console on a badge that has no debug probe.
//!
//! The badge ships without an SWD header populated, so the only channels out of
//! the firmware are three LEDs and, now, this. It enumerates as a USB serial
//! device (`/dev/ttyACM0` on Linux) and prints whatever [`log!`] is handed.
//!
//! The design constraint is that logging must never change the timing of the
//! thing it is observing. Playback is driven by an audio clock and a panel that
//! takes seconds per refresh; a logger that blocked the caller while the host
//! drained a packet would move exactly the numbers we are trying to measure.
//! So:
//!
//! * [`log!`] formats into a fixed [`heapless::String`] and `try_send`s it into
//!   a bounded queue. A full queue drops the line -- it never waits.
//! * The queue is drained by a separate task. If no host has opened the port
//!   (`DTR` deasserted) the line is discarded there instead of piling up.
//! * Every packet write is wrapped in a timeout, so a host that enumerates the
//!   device and then stops reading cannot wedge the drain task for good.
//!
//! Consequently the log is lossy by construction. It is a debugging aid, not a
//! transcript: bursts get truncated and early boot lines are gone before the
//! host opens the port. [`crate::usblog::run`] gives the host a grace period
//! after enumeration for that reason.
//!
//! ## USBD needs HFXO
//!
//! The USB peripheral derives its 48 MHz from the 32 MHz crystal via the PLL.
//! On HFINT (the internal RC oscillator, which is what `embassy_nrf::init`
//! selects by default) USBD powers up and then simply never enumerates -- no
//! error, no interrupt, nothing on the bus. [`run`] therefore starts HFXO
//! itself, with a timeout, and refuses to bring USB up if the crystal does not
//! start. Booting is never blocked on it.

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, pac, peripherals, usb, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use static_cell::StaticCell;

/// Longest single log line. Anything past this is silently truncated by
/// `heapless::String`'s `fmt::Write`, which is preferable to a heap or a
/// panic in a logger.
pub const LINE_LEN: usize = 128;

/// Lines that may be queued before new ones are dropped.
///
/// Costs `QUEUE_LEN * (LINE_LEN + overhead)` bytes of static RAM, so it is
/// deliberately small. It only has to cover a burst -- the drain task empties
/// it at USB full-speed, which is orders of magnitude faster than anything
/// here produces lines.
const QUEUE_LEN: usize = 32;

/// Maximum packet size for the bulk endpoints. 64 is the full-speed maximum.
const MAX_PACKET_SIZE: u16 = 64;

/// How long a single packet write may take before the drain task gives up on
/// the current line. A host that opened the port and then stopped reading
/// stalls the endpoint; without this the drain task would wait forever and the
/// queue would stay full for the rest of the session.
const WRITE_TIMEOUT: Duration = Duration::from_millis(250);

pub type Line = heapless::String<LINE_LEN>;

static QUEUE: Channel<CriticalSectionRawMutex, Line, QUEUE_LEN> = Channel::new();

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

/// Milliseconds since boot, used as the line prefix.
///
/// Public because [`log!`] expands in the caller's crate and needs to reach it.
#[doc(hidden)]
pub fn uptime_ms() -> u64 {
    Instant::now().as_millis()
}

/// Queue an already-formatted line. Drops it if the queue is full.
///
/// Public for the same reason as [`uptime_ms`].
#[doc(hidden)]
pub fn push(mut line: Line) {
    // A terminal that was opened with the usual line settings wants CRLF; the
    // trailing pair is added here so no call site has to remember it.
    let _ = line.push_str("\r\n");
    let _ = QUEUE.try_send(line);
}

/// Print a line to the USB serial console.
///
/// Takes `format_args!` syntax and is safe to call from any task, including
/// interrupt-priority ones -- the queue is guarded by a critical section and
/// the call never awaits. Lines are dropped when the queue is full or when no
/// host is listening.
///
/// ```ignore
/// log!("frame {} took {} ms", index, elapsed);
/// ```
#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {{
        let mut line = $crate::usblog::Line::new();
        // Fully-qualified so the macro works without `core::fmt::Write` being
        // imported at the call site.
        let _ = ::core::fmt::Write::write_fmt(
            &mut line,
            ::core::format_args!("[{:>8}] ", $crate::usblog::uptime_ms()),
        );
        let _ = ::core::fmt::Write::write_fmt(&mut line, ::core::format_args!($($arg)*));
        $crate::usblog::push(line);
    }};
}

/// Bring up USB serial logging.
///
/// Starts HFXO, and on success spawns the USB task. Returns whether USB came
/// up: `false` means the crystal did not start and the firmware is running on
/// HFINT, where USBD cannot enumerate. Either way this returns promptly and
/// boot continues -- a badge with a dead crystal still plays the video.
///
/// Enumeration itself happens asynchronously in the spawned task, and the host
/// only opens the port some time after that, so callers that want their first
/// lines to be seen should wait a couple of seconds before emitting them.
pub async fn run(usbd: Peri<'static, peripherals::USBD>, spawner: &Spawner) -> bool {
    if !start_hfxo().await {
        defmt::warn!("HFXO did not start; USB logging unavailable");
        return false;
    }
    spawner.must_spawn(usb_task(usbd));
    true
}

/// Start the 32 MHz crystal and wait for it, with a cap.
///
/// `embassy_nrf::init` is left on its default (HFINT) so that a dead or badly
/// soldered crystal cannot hang the boot inside the HAL. Requesting HFXO here
/// instead keeps the failure recoverable: on timeout the request is cancelled
/// so the controller stops driving a crystal that is not oscillating, and the
/// chip carries on at HFINT.
async fn start_hfxo() -> bool {
    let clock = pac::CLOCK;

    // Already running (a warm boot, or a future change to the init config).
    if clock.hfclkstat().read().state() {
        return true;
    }

    clock.events_hfclkstarted().write_value(0);
    clock.tasks_hfclkstart().write_value(1);

    // A healthy nRF52840 crystal starts in ~360 us; 100 ms is pure slack.
    for _ in 0..100u16 {
        if clock.events_hfclkstarted().read() != 0 {
            return true;
        }
        Timer::after_millis(1).await;
    }

    clock.tasks_hfclkstop().write_value(1);
    false
}

#[embassy_executor::task]
async fn usb_task(usbd: Peri<'static, peripherals::USBD>) {
    // Hardware VBUS detection: the POWER peripheral raises USBDETECTED /
    // USBPWRRDY when a cable is plugged in, so the stack idles harmlessly on
    // battery power and enumerates if a cable arrives later. Nothing else in
    // this firmware wants CLOCK_POWER, so unlike the badge's mesh firmware
    // there is no interrupt conflict to work around here.
    let driver = Driver::new(usbd, Irqs, HardwareVbusDetect::new(Irqs));

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("BornHack");
    config.product = Some("aegg-apple log");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // CDC-ACM is two interfaces (control + data) tied together by an IAD, so
    // the device descriptor has to advertise the misc/IAD class triple.
    // Without this Windows binds a driver to the control interface alone and
    // the port never appears.
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // embassy-usb's descriptor writer panics on overflow rather than
    // truncating, so these are sized with headroom over the ~75 bytes a
    // single-function CDC-ACM device actually assembles.
    static CONFIG_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 32]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 128]),
        BOS_DESC.init([0; 32]),
        &mut [], // no MS-OS descriptors
        CONTROL_BUF.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), MAX_PACKET_SIZE);
    let mut usb = builder.build();
    let (mut sender, _receiver) = class.split();

    // The device stack and the drain loop are joined rather than spawned
    // separately because the class borrows from the builder; neither ever
    // returns.
    join(usb.run(), async {
        loop {
            let line = QUEUE.receive().await;

            // No terminal has the port open. Discarding here (rather than
            // waiting for a connection) is what keeps the queue from filling
            // with stale lines on a badge running from battery.
            if !sender.dtr() {
                continue;
            }

            if write_line(&mut sender, line.as_bytes()).await.is_err() {
                // Host went away mid-line. Nothing to recover -- the next
                // line starts a fresh packet.
                continue;
            }
        }
    })
    .await;
}

/// Write one line as a sequence of full-speed packets.
async fn write_line<'d, D: embassy_usb::driver::Driver<'d>>(
    sender: &mut embassy_usb::class::cdc_acm::Sender<'d, D>,
    bytes: &[u8],
) -> Result<(), EndpointError> {
    let mut last_full = false;
    for chunk in bytes.chunks(MAX_PACKET_SIZE as usize) {
        match with_timeout(WRITE_TIMEOUT, sender.write_packet(chunk)).await {
            Ok(r) => r?,
            Err(_) => return Err(EndpointError::Disabled),
        }
        last_full = chunk.len() == MAX_PACKET_SIZE as usize;
    }
    // A transfer that ends on a full packet needs a zero-length packet to
    // terminate it, or the host holds the data back waiting for more.
    if last_full {
        match with_timeout(WRITE_TIMEOUT, sender.write_packet(&[])).await {
            Ok(r) => r?,
            Err(_) => return Err(EndpointError::Disabled),
        }
    }
    Ok(())
}

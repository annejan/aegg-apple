use core::{fmt::Debug, future::Future};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;

// Section 15.2 of the HINK-E0213A07 data sheet says to hold for 10ms
const RESET_DELAY_MS: u64 = 10;

/// Trait implemented by displays to provide implementation of core functionality.
pub trait DisplayInterface {
    type Error;

    /// Send a command to the controller.
    ///
    /// Prefer calling `execute` on a [Command](../command/enum.Command.html) over calling this
    /// directly.
    fn send_command(&mut self, command: u8) -> impl Future<Output = Result<(), Self::Error>>;

    /// Send data for a command.
    fn send_data(&mut self, data: &[u8]) -> impl Future<Output = Result<(), Self::Error>>;

    /// Reset the controller.
    fn reset(&mut self) -> impl Future<Output = ()>;

    /// Wait for the controller to indicate it is not busy.
    ///
    /// Use when BUSY is already known to be high (e.g. after reset or soft reset).
    fn busy_wait(&mut self) -> impl Future<Output = Result<(), Self::Error>>;

    /// Wait for a triggered display operation to complete.
    ///
    /// Use this after sending the UpdateDisplay (0x20) command. The controller
    /// may not assert BUSY=HIGH immediately, so calling busy_wait() directly
    /// risks seeing BUSY=LOW and returning before the operation starts.
    /// Waits until BUSY is HIGH (tolerating it already being high), then waits
    /// for BUSY to return LOW (operation complete).
    fn busy_wait_for_completion(&mut self) -> impl Future<Output = Result<(), Self::Error>>;

    /// Receive `buf.len()` bytes from the display after a command has been sent.
    ///
    /// Called immediately after [`send_command`] to read a register response
    /// (e.g. command 0x33 — Read LUT Register).  The default implementation is
    /// a no-op that leaves `buf` unchanged; platforms where MISO is wired should
    /// override this.
    fn receive_data(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<(), Self::Error>> {
        let _ = buf;
        async { Ok(()) }
    }
}

/// The hardware interface to a display.
///
/// ### Example
///
/// This example uses the Linux implementation of the embedded HAL traits to build a display
/// interface. For a complete example see [the Raspberry Pi Inky pHAT example](https://github.com/wezm/ssd1675/blob/master/examples/raspberry_pi_inky_phat.rs).
///
/// ```ignore
/// extern crate linux_embedded_hal;
/// use linux_embedded_hal::spidev::{self, SpidevOptions};
/// use linux_embedded_hal::sysfs_gpio::Direction;
/// use linux_embedded_hal::Delay;
/// use linux_embedded_hal::{Pin, Spidev};
///
/// extern crate ssd1675;
/// use ssd1675::{Builder, Dimensions, Display, GraphicDisplay, Rotation};
///
/// // Configure SPI
/// let mut spi = Spidev::open("/dev/spidev0.0").expect("SPI device");
/// let options = SpidevOptions::new()
///     .bits_per_word(8)
///     .max_speed_hz(4_000_000)
///     .mode(spidev::SPI_MODE_0)
///     .build();
/// spi.configure(&options).expect("SPI configuration");
///
/// // https://pinout.xyz/pinout/inky_phat
/// // Configure Digital I/O Pins
///
/// let busy = Pin::new(17); // BCM17
/// busy.export().expect("busy export");
/// while !busy.is_exported() {}
/// busy.set_direction(Direction::In).expect("busy Direction");
///
/// let dc = Pin::new(22); // BCM22
/// dc.export().expect("dc export");
/// while !dc.is_exported() {}
/// dc.set_direction(Direction::Out).expect("dc Direction");
/// dc.set_value(1).expect("dc Value set to 1");
///
/// let reset = Pin::new(27); // BCM27
/// reset.export().expect("reset export");
/// while !reset.is_exported() {}
/// reset
///     .set_direction(Direction::Out)
///     .expect("reset Direction");
/// reset.set_value(1).expect("reset Value set to 1");
///
/// // Build the interface from the pins and SPI device
/// let controller = ssd1675::Interface::new(spi, busy, dc, reset);
///
#[allow(dead_code)] // Prevent warning about fields being unused
pub struct Interface<SpiDev, BUSY, DC, RESET>
where
    SpiDev: SpiDevice<u8>,
{
    /// SPI Device interface (chip select is owned by the SpiDevice)
    spi: SpiDev,
    /// Active low busy pin (input)
    busy: BUSY,
    /// Data/Command Control Pin (High for data, Low for command) (output)
    dc: DC,
    /// Pin for resetting the controller (output)
    reset: RESET,
}

impl<SpiDev, BUSY, DC, RESET> Interface<SpiDev, BUSY, DC, RESET>
where
    SpiDev: SpiDevice<u8>,
    BUSY: Wait,
    DC: OutputPin,
    RESET: OutputPin,
{
    /// Create a new Interface from embedded hal traits.
    pub fn new(spi: SpiDev, busy: BUSY, dc: DC, reset: RESET) -> Self {
        Self {
            spi,
            busy,
            dc,
            reset,
        }
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), SpiDev::Error> {
        // Linux has a default limit of 4096 bytes per SPI transfer
        // https://github.com/torvalds/linux/blob/ccda4af0f4b92f7b4c308d3acc262f4a7e3affad/drivers/spi/spidev.c#L93
        if cfg!(target_os = "linux") {
            for data_chunk in data.chunks(4096) {
                self.spi.write(data_chunk).await?;
            }
        } else {
            self.spi.write(data).await?;
        }

        Ok(())
    }
}

/// Longest wait for BUSY to rise after a refresh is triggered.
const BUSY_ASSERT_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(500);
/// Longest wait for an in-flight refresh to finish.
const BUSY_CLEAR_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(12);

impl<SpiDev, BUSY, DC, RESET> DisplayInterface for Interface<SpiDev, BUSY, DC, RESET>
where
    SpiDev: SpiDevice<u8>,
    SpiDev::Error: Debug,
    BUSY: Wait,
    DC: OutputPin,
    DC::Error: Debug,
    RESET: OutputPin,
    RESET::Error: Debug,
{
    type Error = SpiDev::Error;

    async fn reset(&mut self) {
        self.reset.set_low().unwrap();
        Timer::after_millis(RESET_DELAY_MS).await;
        self.reset.set_high().unwrap();
        // Allow time for the display's internal power-on reset to complete.
        Timer::after_millis(RESET_DELAY_MS).await;
    }

    async fn send_command(&mut self, command: u8) -> Result<(), SpiDev::Error> {
        self.dc.set_low().unwrap();
        self.write(&[command]).await?;
        self.dc.set_high().unwrap();

        Ok(())
    }

    async fn send_data(&mut self, data: &[u8]) -> Result<(), SpiDev::Error> {
        self.dc.set_high().unwrap();
        self.write(data).await
    }

    async fn busy_wait(&mut self) -> Result<(), SpiDev::Error> {
        // Bounded: a wedged BUSY would otherwise hang the caller forever,
        // which on a badge with no debug probe is indistinguishable from a
        // crash and cannot be diagnosed.
        let _ = embassy_time::with_timeout(BUSY_CLEAR_TIMEOUT, self.busy.wait_for_low()).await;
        Ok(())
    }

    async fn busy_wait_for_completion(&mut self) -> Result<(), SpiDev::Error> {
        // See BUSY_CLEAR_TIMEOUT: both waits below are bounded.
        // wait_for_high() handles the race: if BUSY already asserted before we
        // get here, it returns immediately rather than missing the edge.
        if embassy_time::with_timeout(BUSY_ASSERT_TIMEOUT, self.busy.wait_for_high())
            .await
            .is_err()
        {
            return Ok(());
        }
        let _ = embassy_time::with_timeout(BUSY_CLEAR_TIMEOUT, self.busy.wait_for_low()).await;
        Ok(())
    }
}

//! Board pin assignments for the nRF52840 CyberAegg badge (Prototype V2).
//!
//! Only the pins this firmware actually touches are listed: the EPD panel,
//! the buzzer, the external QSPI flash, the Fire button and the three status
//! LEDs.  The full badge pinout (joystick, LoRa, NFC, QWIIC, battery) lives in
//! the upstream `bornhack-firmware-2026` repo.

/// Extract a board pin by name.
///
/// Pins configured for **Cyber Ægg Prototype V2 board**.
///
/// # Example usage
///
/// ```ignore
/// let buzzer_pin = board!(p, buzzer);
/// ```
#[macro_export]
#[rustfmt::skip]
macro_rules! board {
    // RGB LED pins (low is on)
    ($p:expr, led_red)    => { $p.P1_07 };
    ($p:expr, led_green)  => { $p.P1_15 };
    ($p:expr, led_blue)   => { $p.P0_02 };

    // EPD display (SSD1675 / SSD1680)
    ($p:expr, epd_busy )  => { $p.P0_14 };
    ($p:expr, epd_reset)  => { $p.P0_11 };
    ($p:expr, epd_dc)     => { $p.P0_12 };
    ($p:expr, epd_csn)    => { $p.P1_09 };
    ($p:expr, epd_sck)    => { $p.P0_08 };
    ($p:expr, epd_mosi)   => { $p.P0_27 };
    ($p:expr, epd_spi)    => { $p.SPI3 };

    // Buzzer pin output (PWM0 drives the piezo)
    ($p:expr, buzzer)     => { $p.P0_13 };
    ($p:expr, buzzer_pwm) => { $p.PWM0 };

    // Joystick fire button (active low) — the only input this firmware uses.
    ($p:expr, joy_fire)   => { $p.P1_02 };

    // External Flash (QSPI flash)
    ($p:expr, flash_csn)  => { $p.P0_25 };
    ($p:expr, flash_sck)  => { $p.P0_21 };
    ($p:expr, flash_io0)  => { $p.P0_20 };
    ($p:expr, flash_io1)  => { $p.P0_24 };
    ($p:expr, flash_io2)  => { $p.P0_22 };
    ($p:expr, flash_io3)  => { $p.P0_23 };
}

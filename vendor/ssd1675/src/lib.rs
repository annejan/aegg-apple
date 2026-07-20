#![no_std]

//! SSD1675 ePaper Display Driver (async embassy port)
//!
//! ### Usage
//!
//! To control a display you will need:
//!
//! * An [Interface] to the controller
//! * A [display configuration][Config]
//! * A [Display]
//!
//! The [Interface] captures the details of the hardware connection to the SSD1675 controller. This
//! includes an SPI device and some GPIO pins. The SSD1675 can control many different displays that
//! vary in dimensions, rotation, and driving characteristics. The [Config] captures these details.
//! To aid in constructing the [Config] there is a [Builder] interface. Finally when you have an
//! interface and a [Config] a [Display] instance can be created.
//!
//! Optionally the [Display] can be promoted to a [GraphicDisplay], which allows it to use the
//! functionality from the [embedded-graphics crate][embedded-graphics]. The plain display only
//! provides the ability to update the display by passing black/white and red buffers.
//!
//! To update the display you will typically follow this flow:
//!
//! 1. [reset](display/struct.Display.html#method.reset)
//! 1. [clear](graphics/struct.GraphicDisplay.html#method.clear)
//! 1. [update](graphics/struct.GraphicDisplay.html#method.update)
//! 1. [sleep](display/struct.Display.html#method.deep_sleep)
//!
//! [Interface]: interface/struct.Interface.html
//! [Display]: display/struct.Display.html
//! [GraphicDisplay]: display/struct.GraphicDisplay.html
//! [Config]: config/struct.Config.html
//! [Builder]: config/struct.Builder.html
//! [embedded-graphics]: https://crates.io/crates/embedded-graphics

// Bridge `std` in on the host for `#[cfg(test)]` modules that use `Vec`/`vec!`.
#[cfg(test)]
extern crate std;

pub mod command;
pub mod config;
pub mod display;
pub mod graphics;
pub mod interface;
pub mod partial;

#[cfg(feature = "staged")]
pub mod staged;

pub use config::Builder;
pub use display::{
    Dimensions, Display, DisplayVariant, LUT_TABLE_MIN_C, LUT_TABLE_SIZE, LUT_TABLE_STEP_C10,
    Rotation, UpdateMode, detect_variant_from_otp, patch_no_invert, waveform_frames,
};
pub use graphics::{Color, GraphicDisplay};
pub use interface::DisplayInterface;
pub use interface::Interface;

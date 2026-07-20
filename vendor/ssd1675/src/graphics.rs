use crate::{
    display::{Display, Rotation, UpdateMode},
    interface::DisplayInterface,
};
use core::{
    convert::{AsMut, AsRef},
    ops::{Deref, DerefMut},
};

/// Represents the state of a pixel in the display.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Color {
    Black,
    White,
    Red,
}

/// A display that holds buffers for drawing into and updating the display from.
///
/// When the `graphics` feature is enabled `GraphicDisplay` implements the `Draw` trait from
/// [embedded-graphics](https://crates.io/crates/embedded-graphics). This allows basic shapes and
/// text to be drawn on the display.
pub struct GraphicDisplay<'a, I, B = &'a mut [u8]>
where
    I: DisplayInterface,
{
    display: Display<'a, I>,
    black_buffer: B,
    red_buffer: B,
    work_buffer: B,
}

impl<'a, I, B> GraphicDisplay<'a, I, B>
where
    I: DisplayInterface,
    B: AsMut<[u8]>,
    B: AsRef<[u8]>,
{
    /// Promote a `Display` to a `GraphicDisplay`.
    ///
    /// B/W, Red, and work buffers for drawing must be supplied. These should be `rows * cols / 8`
    /// bytes in length.
    pub fn new(display: Display<'a, I>, black_buffer: B, red_buffer: B, work_buffer: B) -> Self {
        GraphicDisplay {
            display,
            black_buffer,
            red_buffer,
            work_buffer,
        }
    }

    /// Full tricolor update using the slow OTP Mode 2 waveform. Supports red pixels.
    ///
    /// `lut_speed` scales the LUT cycle-duration bytes (`100` = OEM, `0` =
    /// no delay). See [`Display::update_tc`].
    pub async fn update_tc(&mut self, lut_speed: u8) -> Result<(), I::Error> {
        // The full-refresh LUT selects each pixel's waveform row from the
        // (RED, BW) RAM-bit pair: (0,0)=black L0, (0,1)=white L1,
        // (1,0)=red L2, (1,1)=IGNORE L3 (no drive).  The graphics buffers
        // encode red as (BW=1, RED=1) — which would land on L3 and leave red
        // pixels undriven (rendered transparent).  Translate into the
        // controller convention by clearing the BW bit wherever red is set,
        // so red becomes (RED=1, BW=0) → L2.  Black/white are unaffected
        // (red bit is 0 there).  `black_buffer`/`red_buffer` are left intact
        // so `sync_from_planes` still reads the graphics convention.
        {
            let black = self.black_buffer.as_ref();
            let red = self.red_buffer.as_ref();
            let bw = self.work_buffer.as_mut();
            for (dst, (&b, &r)) in bw.iter_mut().zip(black.iter().zip(red.iter())) {
                *dst = b & !r;
            }
        }
        self.display
            .update_tc(self.work_buffer.as_ref(), self.red_buffer.as_ref(), lut_speed)
            .await
    }

    /// Fast B&W update using the OTP LUT. No red support.
    ///
    /// `lut_speed` scales the LUT cycle-duration bytes (`100` = OEM, `0` =
    /// no delay). See [`Display::update_bw`].
    pub async fn update_bw(&mut self, mode: UpdateMode, lut_speed: u8) -> Result<(), I::Error> {
        self.display
            .update_bw(self.black_buffer.as_ref(), self.red_buffer.as_ref(), mode, lut_speed)
            .await
    }

    /// Update a partial region of the display.
    pub async fn partial_update(
        &mut self,
        start_x_px: u16,
        start_y_px: u16,
        width_px: u16,
        height_px: u16,
    ) -> Result<(), I::Error> {
        let work_buf_ref = self.work_buffer.as_mut();
        let sub_image = make_sub_image(
            self.black_buffer.as_ref(),
            work_buf_ref,
            self.display.cols_as_bytes(),
            start_x_px,
            start_y_px,
            width_px,
            height_px,
        );
        self.display
            .partial_update(sub_image, start_x_px, start_y_px, width_px, height_px)
            .await
    }

    /// Clear the buffers, filling them with a single color.
    pub fn clear(&mut self, color: Color) {
        let (black, red) = match color {
            Color::White => (0xFF, 0x00),
            Color::Black => (0x00, 0x00),
            Color::Red => (0xFF, 0xFF),
        };

        for byte in self.black_buffer.as_mut().iter_mut() {
            *byte = black;
        }

        for byte in self.red_buffer.as_mut().iter_mut() {
            *byte = red;
        }
    }

    /// Mutable access to the black (B/W) framebuffer.
    pub fn black_buffer_mut(&mut self) -> &mut [u8] {
        self.black_buffer.as_mut()
    }

    /// Mutable access to the red framebuffer.
    pub fn red_buffer_mut(&mut self) -> &mut [u8] {
        self.red_buffer.as_mut()
    }

    /// Mutable access to the work buffer (free during the draw phase).
    pub fn work_buffer_mut(&mut self) -> &mut [u8] {
        self.work_buffer.as_mut()
    }

    /// Mutable access to all three buffers at once (avoids borrow conflicts).
    pub fn all_buffers_mut(&mut self) -> (&mut [u8], &mut [u8], &mut [u8]) {
        (self.black_buffer.as_mut(), self.red_buffer.as_mut(), self.work_buffer.as_mut())
    }

    /// Copy raw bitmap data directly into the black and/or red buffers.
    ///
    /// Bytes are copied 1:1 (MSB-first, row-major).
    /// `Some(data)` copies the data; `None` clears the buffer to zero.
    pub fn blit(&mut self, black: Option<&[u8]>, red: Option<&[u8]>) {
        let buf = self.black_buffer.as_mut();

        match black {
            Some(data) => {
                let len = data.len().min(buf.len());
                buf[..len].copy_from_slice(&data[..len]);
            }
            None => buf.fill(0),
        }

        let buf = self.red_buffer.as_mut();
        match red {
            Some(data) => {
                let len = data.len().min(buf.len());
                buf[..len].copy_from_slice(&data[..len]);
            }
            None => buf.fill(0),
        }
    }

    fn set_pixel(&mut self, x: u32, y: u32, color: Color) {
        let (index, bit) = rotation(
            x,
            y,
            self.cols() as u32,
            self.rows() as u32,
            self.rotation(),
        );
        let index = index as usize;

        match color {
            Color::Black => {
                self.black_buffer.as_mut()[index] &= !bit;
                self.red_buffer.as_mut()[index] &= !bit;
            }
            Color::White => {
                self.black_buffer.as_mut()[index] |= bit;
                self.red_buffer.as_mut()[index] &= !bit;
            }
            Color::Red => {
                self.black_buffer.as_mut()[index] |= bit;
                self.red_buffer.as_mut()[index] |= bit;
            }
        }
    }
}

impl<'a, I, B> Deref for GraphicDisplay<'a, I, B>
where
    I: DisplayInterface,
{
    type Target = Display<'a, I>;

    fn deref(&self) -> &Display<'a, I> {
        &self.display
    }
}

impl<'a, I, B> DerefMut for GraphicDisplay<'a, I, B>
where
    I: DisplayInterface,
{
    fn deref_mut(&mut self) -> &mut Display<'a, I> {
        &mut self.display
    }
}

fn rotation(x: u32, y: u32, width: u32, height: u32, rotation: Rotation) -> (u32, u8) {
    match rotation {
        Rotation::Rotate0 => (x / 8 + (width / 8) * y, 0x80 >> (x % 8)),
        Rotation::Rotate90 => ((width - 1 - y) / 8 + (width / 8) * x, 0x01 << (y % 8)),
        Rotation::Rotate180 => (
            ((width / 8) * height - 1) - (x / 8 + (width / 8) * y),
            0x01 << (x % 8),
        ),
        Rotation::Rotate270 => (y / 8 + (height - 1 - x) * (width / 8), 0x80 >> (y % 8)),
    }
}

#[allow(clippy::indexing_slicing)]
fn make_sub_image<'a>(
    black_buffer: &[u8],
    work_buffer: &'a mut [u8],
    display_width_as_bytes: u8,
    start_x_px: u16,
    start_y_px: u16,
    width_px: u16,
    height_px: u16,
) -> &'a [u8] {
    let mut at = 0_usize;
    let start_x_bytes = start_x_px / 8;
    let width_bytes = width_px / 8;
    let end_y_px = start_y_px + height_px;
    for i in start_y_px..end_y_px {
        let start_x = ((i * display_width_as_bytes as u16) + start_x_bytes) as usize;
        let end_x = start_x + width_bytes as usize;
        for b in black_buffer.iter().take(end_x).skip(start_x) {
            work_buffer[at] = *b;
            at += 1;
        }
    }
    let num_bytes = (width_bytes * height_px) as usize;
    &work_buffer[0..num_bytes]
}

#[cfg(feature = "graphics")]
use embedded_graphics::pixelcolor::raw::RawU8;
#[cfg(feature = "graphics")]
use embedded_graphics::prelude::*;

#[cfg(feature = "graphics")]
impl PixelColor for Color {
    type Raw = RawU8;
}

#[cfg(feature = "graphics")]
impl<I, B> DrawTarget for GraphicDisplay<'_, I, B>
where
    I: DisplayInterface,
    B: AsMut<[u8]>,
    B: AsRef<[u8]>,
{
    type Color = Color;
    type Error = core::convert::Infallible;

    fn draw_iter<Iter>(&mut self, pixels: Iter) -> Result<(), Self::Error>
    where
        Iter: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let sz = self.size();
        for Pixel(Point { x, y }, color) in pixels {
            let x = x as u32;
            let y = y as u32;
            if x < sz.width && y < sz.height {
                self.set_pixel(x, y, color)
            }
        }
        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<I, B> OriginDimensions for GraphicDisplay<'_, I, B>
where
    I: DisplayInterface,
{
    fn size(&self) -> Size {
        match self.rotation() {
            Rotation::Rotate0 | Rotation::Rotate180 => {
                Size::new(self.cols().into(), self.rows().into())
            }
            Rotation::Rotate90 | Rotation::Rotate270 => {
                Size::new(self.rows().into(), self.cols().into())
            }
        }
    }
}

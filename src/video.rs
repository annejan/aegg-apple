//! Bit-run-length frame decoder.
//!
//! Matches the encoder in `tools/encode.py`. `BADAPPLE.VID` is a 20-byte
//! header, then a table of `frame_count + 1` little-endian byte offsets, then
//! the coded frames back to back. The trailing offset marks the end of the
//! last frame, so a frame's compressed length is always
//! `offset[n + 1] - offset[n]`.
//!
//! Frames are coded independently, with no reference to the frame before.
//! That is what lets the player jump straight to whichever frame the audio
//! clock currently calls for -- when the panel takes longer to refresh than
//! the frame interval, the player skips ahead instead of falling behind.

pub const MAGIC: &[u8; 8] = b"AEGGVID1";
pub const HEADER_LEN: usize = 20;

pub const WIDTH: usize = 152;
pub const HEIGHT: usize = 152;
pub const STRIDE: usize = WIDTH / 8;
pub const FRAME_BYTES: usize = STRIDE * HEIGHT;

/// Parsed `BADAPPLE.VID` header.
#[derive(Clone, Copy, Debug)]
pub struct VideoHeader {
    pub width: u16,
    pub height: u16,
    pub fps: u16,
    pub frame_count: u32,
}

impl VideoHeader {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < HEADER_LEN || &buf[..8] != MAGIC {
            return None;
        }
        let hdr = Self {
            width: u16::from_le_bytes(buf[8..10].try_into().ok()?),
            height: u16::from_le_bytes(buf[10..12].try_into().ok()?),
            fps: u16::from_le_bytes(buf[12..14].try_into().ok()?),
            frame_count: u32::from_le_bytes(buf[16..20].try_into().ok()?),
        };
        // The player's framebuffer is a fixed 152x152; a file built for any
        // other geometry would decode into the wrong shape.
        if hdr.width as usize != WIDTH || hdr.height as usize != HEIGHT {
            return None;
        }
        if hdr.fps == 0 || hdr.frame_count == 0 {
            return None;
        }
        Some(hdr)
    }

    /// Byte offset of the entry for `frame` in the offset table.
    pub fn offset_entry(&self, frame: u32) -> u32 {
        HEADER_LEN as u32 + frame * 4
    }

    /// Duration of the whole clip in milliseconds.
    pub fn duration_ms(&self) -> u32 {
        self.frame_count * 1000 / self.fps as u32
    }
}

/// Decode a bit-run-length coded frame into a 1bpp plane.
///
/// The first byte is the value of the leading bit; each run length follows,
/// one byte when under 128, otherwise two bytes `0x80 | hi7` then `lo8`.
/// Returns `None` if the runs do not describe exactly `FRAME_BYTES * 8`
/// pixels, which would mean a corrupt or truncated file.
pub fn decode_frame(enc: &[u8], out: &mut [u8; FRAME_BYTES]) -> Option<()> {
    out.fill(0);

    let first = *enc.first()?;
    if first > 1 {
        return None;
    }

    let mut bit = first != 0;
    let mut pos: usize = 0;
    let mut i = 1;
    const TOTAL_BITS: usize = FRAME_BYTES * 8;

    while i < enc.len() {
        let b = enc[i];
        i += 1;
        let run = if b & 0x80 != 0 {
            let lo = *enc.get(i)?;
            i += 1;
            (((b & 0x7F) as usize) << 8) | lo as usize
        } else {
            b as usize
        };

        let end = pos.checked_add(run)?;
        if end > TOTAL_BITS {
            return None;
        }

        if bit {
            set_bits(out, pos, end);
        }

        pos = end;
        bit = !bit;
    }

    (pos == TOTAL_BITS).then_some(())
}

/// Set bits `[from, to)` in an MSB-first packed bitmap.
///
/// Whole bytes in the middle are filled at once; only the partial bytes at
/// each end are touched bit by bit. At 4 fps the panel gives us ~250 ms per
/// frame, but this still runs while audio DMA is being refilled, so it is
/// worth not doing 23104 individual bit writes.
fn set_bits(out: &mut [u8; FRAME_BYTES], from: usize, to: usize) {
    if from >= to {
        return;
    }

    let first_byte = from / 8;
    let last_byte = (to - 1) / 8;

    if first_byte == last_byte {
        let mask = (0xFFu8 >> (from % 8)) & !(0x7Fu8 >> ((to - 1) % 8));
        out[first_byte] |= mask;
        return;
    }

    out[first_byte] |= 0xFFu8 >> (from % 8);
    for byte in &mut out[first_byte + 1..last_byte] {
        *byte = 0xFF;
    }
    out[last_byte] |= !(0x7Fu8 >> ((to - 1) % 8));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode with the same scheme as tools/encode.py, for round-trip tests.
    fn encode(plane: &[u8; FRAME_BYTES]) -> alloc::vec::Vec<u8> {
        let bit = |i: usize| (plane[i / 8] >> (7 - i % 8)) & 1;
        let mut out = alloc::vec::Vec::new();
        let mut cur = bit(0);
        out.push(cur);
        let mut run = 1usize;
        for i in 1..FRAME_BYTES * 8 {
            if bit(i) == cur {
                run += 1;
            } else {
                emit(&mut out, run);
                cur = bit(i);
                run = 1;
            }
        }
        emit(&mut out, run);
        out
    }

    fn emit(out: &mut alloc::vec::Vec<u8>, run: usize) {
        if run < 128 {
            out.push(run as u8);
        } else {
            out.push(0x80 | (run >> 8) as u8);
            out.push((run & 0xFF) as u8);
        }
    }

    extern crate alloc;
    extern crate std;

    #[test]
    fn round_trips_all_white() {
        let plane = [0xFFu8; FRAME_BYTES];
        let enc = encode(&plane);
        let mut out = [0u8; FRAME_BYTES];
        assert!(decode_frame(&enc, &mut out).is_some());
        assert_eq!(out, plane);
    }

    #[test]
    fn round_trips_all_black() {
        let plane = [0x00u8; FRAME_BYTES];
        let enc = encode(&plane);
        let mut out = [0xAAu8; FRAME_BYTES];
        assert!(decode_frame(&enc, &mut out).is_some());
        assert_eq!(out, plane);
    }

    #[test]
    fn round_trips_a_pattern_with_long_and_short_runs() {
        let mut plane = [0u8; FRAME_BYTES];
        // Long white band, then fine alternating detail: exercises both the
        // one-byte and two-byte run encodings and the partial-byte edges.
        for byte in plane.iter_mut().take(FRAME_BYTES / 2) {
            *byte = 0xFF;
        }
        for (i, byte) in plane.iter_mut().skip(FRAME_BYTES / 2).enumerate() {
            *byte = if i % 2 == 0 { 0b1010_1010 } else { 0b0011_0011 };
        }
        let enc = encode(&plane);
        let mut out = [0u8; FRAME_BYTES];
        assert!(decode_frame(&enc, &mut out).is_some());
        assert_eq!(out, plane);
    }

    #[test]
    fn rejects_a_truncated_frame() {
        let plane = [0xFFu8; FRAME_BYTES];
        let mut enc = encode(&plane);
        enc.pop();
        let mut out = [0u8; FRAME_BYTES];
        assert!(decode_frame(&enc, &mut out).is_none());
    }

    #[test]
    fn rejects_runs_past_the_end_of_the_frame() {
        // Leading bit 0, then a run longer than the whole frame.
        let enc = [0x00u8, 0xFF, 0xFF];
        let mut out = [0u8; FRAME_BYTES];
        assert!(decode_frame(&enc, &mut out).is_none());
    }

    #[test]
    fn rejects_a_bad_leading_bit() {
        let mut out = [0u8; FRAME_BYTES];
        assert!(decode_frame(&[0x02, 0x01], &mut out).is_none());
    }

    #[test]
    fn header_rejects_wrong_geometry() {
        let mut buf = [0u8; HEADER_LEN];
        buf[..8].copy_from_slice(MAGIC);
        buf[8..10].copy_from_slice(&200u16.to_le_bytes());
        buf[10..12].copy_from_slice(&200u16.to_le_bytes());
        buf[12..14].copy_from_slice(&4u16.to_le_bytes());
        buf[16..20].copy_from_slice(&10u32.to_le_bytes());
        assert!(VideoHeader::parse(&buf).is_none());
    }
}

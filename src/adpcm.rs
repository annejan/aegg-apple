//! 4-bit IMA ADPCM decoder.
//!
//! Matches the encoder in `tools/encode.py`. The stream is a 20-byte header
//! followed by fixed-size blocks. Each block carries its own starting
//! predictor and step index, so any block decodes without the ones before it
//! and playback can start or resync at a block boundary.

pub const MAGIC: &[u8; 8] = b"AEGGSND1";
pub const HEADER_LEN: usize = 20;

/// Bytes of block header ahead of the packed nibbles.
const BLOCK_HEADER_LEN: usize = 4;

const STEP_TABLE: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66,
    73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
    494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272,
    2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493,
    10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
];

const INDEX_TABLE: [i8; 16] = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8];

/// Parsed `BADAPPLE.SND` header.
#[derive(Clone, Copy, Debug)]
pub struct SoundHeader {
    pub sample_rate: u32,
    pub sample_count: u32,
    pub block_samples: u16,
    pub block_bytes: u16,
}

impl SoundHeader {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < HEADER_LEN || &buf[..8] != MAGIC {
            return None;
        }
        let hdr = Self {
            sample_rate: u32::from_le_bytes(buf[8..12].try_into().ok()?),
            sample_count: u32::from_le_bytes(buf[12..16].try_into().ok()?),
            block_samples: u16::from_le_bytes(buf[16..18].try_into().ok()?),
            block_bytes: u16::from_le_bytes(buf[18..20].try_into().ok()?),
        };
        // A zero block size would make block_offset divide by zero and a
        // zero rate would make the audio clock meaningless.
        if hdr.block_bytes as usize <= BLOCK_HEADER_LEN
            || hdr.block_samples == 0
            || hdr.sample_rate == 0
        {
            return None;
        }
        Some(hdr)
    }

    /// Byte offset of a block within the file.
    pub fn block_offset(&self, block: u32) -> u32 {
        HEADER_LEN as u32 + block * self.block_bytes as u32
    }

    pub fn block_count(&self) -> u32 {
        self.sample_count.div_ceil(self.block_samples as u32)
    }
}

/// Decode one block into `out`, returning how many samples were written.
///
/// Writes at most `out.len()` samples. Trailing nibbles that would not fit
/// are dropped rather than silently wrapping.
pub fn decode_block(block: &[u8], out: &mut [i16]) -> usize {
    if block.len() < BLOCK_HEADER_LEN || out.is_empty() {
        return 0;
    }

    let mut pred = i16::from_le_bytes([block[0], block[1]]) as i32;
    let mut index = block[2].min(88) as usize;

    out[0] = pred as i16;
    let mut n = 1;

    'outer: for &byte in &block[BLOCK_HEADER_LEN..] {
        for code in [byte & 0x0F, byte >> 4] {
            if n >= out.len() {
                break 'outer;
            }
            let step = STEP_TABLE[index];

            // Reconstruct the magnitude the encoder searched for: step/8
            // plus step/2, step/4, step/8 for each set magnitude bit.
            let mut delta = step >> 3;
            if code & 4 != 0 {
                delta += step;
            }
            if code & 2 != 0 {
                delta += step >> 1;
            }
            if code & 1 != 0 {
                delta += step >> 2;
            }

            pred += if code & 8 != 0 { -delta } else { delta };
            pred = pred.clamp(i16::MIN as i32, i16::MAX as i32);

            index = (index as i32 + INDEX_TABLE[code as usize] as i32).clamp(0, 88) as usize;

            out[n] = pred as i16;
            n += 1;
        }
    }

    n
}

/// Playback gain, applied before the sample is mapped to a duty cycle.
///
/// A piezo is a poor loudspeaker: no cone, no enclosure, and a sharp
/// mechanical resonance well above the fundamentals of most music, so a
/// faithfully reproduced waveform is barely audible. Gain here clips the
/// loud parts, which is a crude limiter -- it raises average power a lot for
/// a modest amount of distortion, and on this transducer that trade is
/// strongly worth it.
pub const GAIN: i32 = 4;

/// Map a signed sample onto a PWM duty value in `0..=top`.
///
/// The piezo is driven single-ended, so silence sits at half scale and the
/// waveform swings either side of it. The result is clamped, so gain above
/// unity clips rather than wrapping -- wrapping would turn a loud passage
/// into noise.
#[inline]
pub fn sample_to_duty(sample: i16, top: u16) -> u16 {
    let mid = top as i32 / 2;
    (mid + (sample as i32 * GAIN * mid) / 32768).clamp(0, top as i32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_rejects_bad_magic() {
        let mut buf = [0u8; HEADER_LEN];
        buf[..8].copy_from_slice(b"NOTASND1");
        assert!(SoundHeader::parse(&buf).is_none());
    }

    #[test]
    fn header_rejects_degenerate_sizes() {
        let mut buf = [0u8; HEADER_LEN];
        buf[..8].copy_from_slice(MAGIC);
        buf[8..12].copy_from_slice(&6000u32.to_le_bytes());
        buf[12..16].copy_from_slice(&1000u32.to_le_bytes());
        // block_samples = 0, block_bytes = 0
        assert!(SoundHeader::parse(&buf).is_none());
    }

    #[test]
    fn silence_decodes_to_the_starting_predictor() {
        // All-zero codes still step the predictor by step/8 each time, so
        // only the first sample is guaranteed. Check that one.
        let mut block = [0u8; 512];
        block[..2].copy_from_slice(&1234i16.to_le_bytes());
        let mut out = [0i16; 1017];
        let n = decode_block(&block, &mut out);
        assert_eq!(n, 1017);
        assert_eq!(out[0], 1234);
    }

    #[test]
    fn decode_respects_a_short_output_buffer() {
        let block = [0u8; 512];
        let mut out = [0i16; 8];
        assert_eq!(decode_block(&block, &mut out), 8);
    }

    #[test]
    fn duty_maps_silence_to_half_scale() {
        assert_eq!(sample_to_duty(0, 2666), 1333);
        assert_eq!(sample_to_duty(i16::MIN, 2666), 0);
        assert!(sample_to_duty(i16::MAX, 2666) <= 2666);
        // Gain clips instead of wrapping.
        assert_eq!(sample_to_duty(i16::MAX / 2, 2666), 2666);
    }
}

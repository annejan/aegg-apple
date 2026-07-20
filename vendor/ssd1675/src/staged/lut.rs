//! Short custom stage LUT encoder.  Produces the LUT body (5 waveform rows ×
//! n_phases selector bytes, then n_phases TP entries) for one stage.

use crate::partial::{Layout, SSD1675A_LAYOUT, SSD1675B_LAYOUT};

/// Maximum phases any supported variant uses (SSD1675B = 10).
pub const MAX_PHASES: usize = 10;
/// Maximum LUT body length any variant honours (SSD1675B = 99).
pub const MAX_BODY: usize = 99;

/// One TP timing entry: sub-phase durations A–D plus repeat count RP.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tp {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub rp: u8,
}

/// Per-group waveform: one selector byte per phase (row of the LUT waveform region).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GroupWaveform {
    pub phases: [u8; MAX_PHASES],
}

/// A short stage LUT: the four drive groups (Black/White/Red/NoOp) + VCOM,
/// shared TP timing, and the active phase count.  `red` is `None` when the
/// stage has no red drive — the encoded LUT2 row is then all-zero (no red drive).
#[derive(Clone, Debug)]
pub struct StageLut {
    pub n_phases: usize,
    pub black: GroupWaveform,
    pub white: GroupWaveform,
    /// `None` ⇒ red group omitted (LUT2 row all-zero).
    pub red: Option<GroupWaveform>,
    pub vcom: GroupWaveform,
    pub tp: [Tp; MAX_PHASES],
}

impl StageLut {
    fn layout(b_variant: bool) -> Layout {
        if b_variant { SSD1675B_LAYOUT } else { SSD1675A_LAYOUT }
    }

    /// Encode this stage into `out`, returning the number of body bytes written
    /// (`layout.body_len`).  Rows: 0=Black(LUT0) 1=White(LUT1) 2=Red(LUT2)
    /// 3=NoOp(LUT3, forced zero) 4=VCOM(LUT4).  Phases beyond `n_phases` are zero.
    ///
    /// # Panics
    ///
    /// In debug builds, panics if `out` is shorter than the variant's body
    /// length (`MAX_BODY` is always large enough).
    pub fn encode(&self, b_variant: bool, out: &mut [u8]) -> usize {
        let l = Self::layout(b_variant);
        debug_assert!(out.len() >= l.body_len, "encode out buffer too small");
        let n = self.n_phases.min(l.n_phases);
        out[..l.body_len].fill(0);

        let rows: [&GroupWaveform; 5] = [
            &self.black,
            &self.white,
            self.red.as_ref().unwrap_or(&ZERO_GROUP),
            &ZERO_GROUP, // LUT3 NoOp — always zero net drive
            &self.vcom,
        ];
        for (row, wf) in rows.iter().enumerate() {
            for phase in 0..n {
                out[l.lut_byte(row, phase)] = wf.phases[phase];
            }
        }
        // TP region: n_phases entries of (A,B,C,D,RP).
        for phase in 0..n {
            let base = l.tp_base + phase * l.tp_stride;
            let tp = &self.tp[phase];
            out[base] = tp.a;
            out[base + 1] = tp.b;
            out[base + 2] = tp.c;
            out[base + 3] = tp.d;
            out[base + 4] = tp.rp;
        }
        l.body_len
    }
}

static ZERO_GROUP: GroupWaveform = GroupWaveform { phases: [0; MAX_PHASES] };

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(red: bool) -> StageLut {
        StageLut {
            n_phases: 2,
            black: GroupWaveform { phases: { let mut p = [0; MAX_PHASES]; p[0] = 0x40; p[1] = 0x40; p } },
            white: GroupWaveform { phases: { let mut p = [0; MAX_PHASES]; p[0] = 0x80; p[1] = 0x80; p } },
            red: if red { Some(GroupWaveform { phases: { let mut p = [0; MAX_PHASES]; p[0] = 0x10; p } }) } else { None },
            vcom: GroupWaveform::default(),
            tp: { let mut t = [Tp::default(); MAX_PHASES]; t[0] = Tp { a: 5, b: 0, c: 0, d: 0, rp: 0 }; t[1] = Tp { a: 5, b: 0, c: 0, d: 0, rp: 0 }; t },
        }
    }

    #[test]
    fn encodes_a_variant_rows_and_tp() {
        let mut buf = [0xAA; MAX_BODY];
        let len = sample(false).encode(false, &mut buf);
        assert_eq!(len, 70);
        // LUT0 (black) phase 0 at byte 0, phase 1 at byte 1.
        assert_eq!(buf[0], 0x40);
        assert_eq!(buf[1], 0x40);
        // LUT1 (white) at row 1 → bytes 7,8.
        assert_eq!(buf[7], 0x80);
        assert_eq!(buf[8], 0x80);
        // TP0 at byte 35.
        assert_eq!(buf[35], 5);
        // Untouched tail inside body is zeroed.
        assert_eq!(buf[34], 0);
    }

    #[test]
    fn red_none_zeroes_lut2_row() {
        let mut buf = [0xAA; MAX_BODY];
        sample(false).encode(false, &mut buf);
        // LUT2 (red) row 2 → bytes 14..21 must all be zero when red is None.
        assert!(buf[14..21].iter().all(|&b| b == 0), "no red drive group");
    }

    #[test]
    fn red_some_writes_lut2_row() {
        let mut buf = [0; MAX_BODY];
        sample(true).encode(false, &mut buf);
        assert_eq!(buf[14], 0x10, "red phase 0 at row 2 byte 14");
    }

    #[test]
    fn encodes_b_variant_body_len_99() {
        let mut buf = [0; MAX_BODY];
        let len = sample(true).encode(true, &mut buf);
        assert_eq!(len, 99);
        assert_eq!(buf[0], 0x40); // LUT0 phase 0
        assert_eq!(buf[10], 0x80); // LUT1 row 1 (10 phases) phase 0
        assert_eq!(buf[50], 5); // TP0 at tp_base 50
    }

    #[test]
    fn lut3_noop_always_zero() {
        let mut buf = [0; MAX_BODY];
        sample(true).encode(false, &mut buf);
        assert!(buf[21..28].iter().all(|&b| b == 0), "LUT3 NoOp row must be zero");
    }
}

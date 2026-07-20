//! Staged neighbourhood-aware drive: short equal-length drive stages that give
//! extra cumulative impulse to edge/isolated pixels.  This module holds the
//! **mechanics** (LUT encoding, plane packing, classification, ledger math, and
//! the stage executor).  All *policy and state* (the ledger buffer, the per-update
//! schedule, the refresh loop) lives in the firmware that drives this crate.

mod classify;
mod executor;
mod ledger;
mod lut;
mod preprocess;
mod schedule;

#[cfg(test)]
mod mock;

pub use classify::{classify, distance_transform, Class};
pub use executor::{trigger_stage, upload_lut, upload_planes, Region};
pub use ledger::ledger_apply_stage;
pub use lut::{GroupWaveform, StageLut, Tp, MAX_BODY, MAX_PHASES};
pub use preprocess::{
    delta_has_red, mark_corrections, pack_content_stage, pack_correction_stage,
    pack_erosion_stage, pack_white_boost_stage,
};
pub use schedule::{
    class_impulse, class_in_stage, EROSION_STAGES, LEDGER_CORRECTION_THRESHOLD, MAX_STAGES,
    STAGE_COUNT, STAGE_LEN_MS,
};

/// Per-pixel action for one stage.  Encodes to the two RAM-plane bits the
/// controller uses to select a waveform group: `group = RED*2 + BW`
/// (see `partial::color_to_ram_bits`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StageAction {
    /// LUT3, `(RED=1, BW=1)` — genuinely zero net drive.
    NoOp,
    /// LUT1, `(RED=0, BW=1)`.
    DriveWhite,
    /// LUT0, `(RED=0, BW=0)`.
    DriveBlack,
    /// LUT2, `(RED=1, BW=0)`.
    DriveRed,
}

impl StageAction {
    /// Returns `(red_bit, bw_bit)` for this action.
    #[inline]
    pub const fn ram_bits(self) -> (bool, bool) {
        match self {
            StageAction::DriveBlack => (false, false),
            StageAction::DriveWhite => (false, true),
            StageAction::DriveRed => (true, false),
            StageAction::NoOp => (true, true),
        }
    }
}

#[cfg(test)]
mod action_tests {
    use super::StageAction;

    #[test]
    fn ram_bits_match_partial_group_mapping() {
        // group = RED*2 + BW must equal the LUT row index from partial::color_to_ram_bits
        let g = |a: StageAction| {
            let (red, bw) = a.ram_bits();
            (red as u8) * 2 + (bw as u8)
        };
        assert_eq!(g(StageAction::DriveBlack), 0, "LUT0");
        assert_eq!(g(StageAction::DriveWhite), 1, "LUT1");
        assert_eq!(g(StageAction::DriveRed), 2, "LUT2");
        assert_eq!(g(StageAction::NoOp), 3, "LUT3 / ignore");
    }
}

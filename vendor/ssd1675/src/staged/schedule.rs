//! Pure, host-testable staged-drive schedule helpers.
//!
//! These functions are deterministic transforms over a [`Class`] and integer
//! stage indices — no hardware, no state.  They live in the crate (rather than
//! the firmware) so they compile and run under `cargo test` on the host target,
//! where the firmware's nRF-only modules cannot link.  The firmware re-exports
//! thin wrappers from `src/fw/epd_staged.rs`.
//!
//! Schedule semantics: a logical B/W update is split into [`STAGE_COUNT`] equal
//! [`STAGE_LEN_MS`]-long drive stages.  Earlier stages drive every dirty class;
//! later stages drive only the classes that need extra cumulative impulse:
//!
//! - S0 — all dirty (every class),
//! - S1 — [`Class::Edge`] + [`Class::Isolated`],
//! - S2 — [`Class::Isolated`] only.

use super::Class;

/// Drive-stage length applied uniformly to every stage, in milliseconds.
/// Single retuning knob for the whole schedule.
pub const STAGE_LEN_MS: u32 = 100;
/// Default stage count for a B/W update.  Hard cap is [`MAX_STAGES`].
pub const STAGE_COUNT: usize = 3;
/// Hard upper bound on stages per logical update.
pub const MAX_STAGES: usize = 4;

/// Number of erosion stages (k = 0..4 inclusive). Isolated pixels receive a pulse
/// in every stage (maximum drive); interior pixels receive pulses in fewer stages.
pub const EROSION_STAGES: usize = 5;

/// Carried-in imbalance magnitude above which a transitioning pixel gets an extra
/// correction stage.  Tunable; default chosen so a pixel must net-drift ~2 full
/// isolated updates (±3 each) in one direction before correcting.
pub const LEDGER_CORRECTION_THRESHOLD: i8 = 6;

// Ledger sign-stability: a flagged pixel (|imbalance| > THRESHOLD) must stay
// same-sign after content stages add at most ±STAGE_COUNT, so the correction
// direction read post-content is correct. Requires THRESHOLD >= STAGE_COUNT.
const _: () = assert!(LEDGER_CORRECTION_THRESHOLD as i16 >= STAGE_COUNT as i16);

/// Whether `class` is driven in stage `stage` (0-based).
///
/// S0 = all dirty, S1 = Edge+Isolated, S2 = Isolated.  Stages at or beyond
/// [`STAGE_COUNT`] drive nothing.
///
/// # Arguments
///
/// * `class` - boundary class of the pixel
/// * `stage` - 0-based stage index within the update
///
/// # Returns
///
/// `true` if a pixel of `class` is driven in `stage`.
#[inline]
pub fn class_in_stage(class: Class, stage: usize) -> bool {
    match stage {
        0 => true,
        1 => matches!(class, Class::Edge | Class::Isolated),
        2 => matches!(class, Class::Isolated),
        _ => false,
    }
}

/// Number of stages `class` participates in across a [`STAGE_COUNT`]-stage update.
///
/// This is the cumulative drive impulse (in stage-units) a class receives:
/// [`Class::Interior`] → 1, [`Class::Edge`] → 2, [`Class::Isolated`] → 3.
///
/// # Arguments
///
/// * `class` - boundary class of the pixel
///
/// # Returns
///
/// The count of stages in which `class` is driven.
#[inline]
pub fn class_impulse(class: Class) -> u8 {
    (0..STAGE_COUNT).filter(|&s| class_in_stage(class, s)).count() as u8
}

#[cfg(test)]
mod tests {
    use super::super::Class::*;
    use super::*;

    #[test]
    fn schedule_impulse_decreases_with_distance() {
        assert_eq!(class_impulse(Interior), 1);
        assert_eq!(class_impulse(Edge), 2);
        assert_eq!(class_impulse(Isolated), 3);
    }

    #[test]
    fn imbalance_ceiling_is_two_stages() {
        let ceiling = (class_impulse(Isolated) - class_impulse(Interior)) as u32 * STAGE_LEN_MS;
        assert_eq!(ceiling, 2 * STAGE_LEN_MS);
    }

    #[test]
    fn stage_count_within_hard_cap() {
        assert!(STAGE_COUNT <= MAX_STAGES);
    }
}

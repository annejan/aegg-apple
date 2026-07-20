//! Per-pixel signed charge ledger (feed-forward *commanded* impulse, not a
//! coulomb counter).  Discharge happens only on a real transition.

/// Add `+impulse` (signed) to `ledger[i]` for a driven pixel, saturating.
/// `to_white` drives positive, `to_black` negative (sign convention is the
/// firmware's; kept consistent across apply/bias).
#[inline]
pub fn ledger_apply_stage(ledger: &mut [i8], i: usize, impulse: i8) {
    ledger[i] = ledger[i].saturating_add(impulse);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_accumulates_and_saturates() {
        let mut l = [0i8; 4];
        ledger_apply_stage(&mut l, 0, 3);
        ledger_apply_stage(&mut l, 0, 3);
        assert_eq!(l[0], 6);
        for _ in 0..100 {
            ledger_apply_stage(&mut l, 1, 100);
        }
        assert_eq!(l[1], i8::MAX, "saturates, no overflow");
    }

    #[test]
    fn static_pixel_keeps_charge_until_transition() {
        // Simulate: drive isolated thrice, never transition → ledger holds, no pulse.
        let mut l = [0i8; 1];
        ledger_apply_stage(&mut l, 0, 1);
        ledger_apply_stage(&mut l, 0, 1);
        ledger_apply_stage(&mut l, 0, 1);
        assert_eq!(l[0], 3);
        // No bias read because no transition occurs → value persists.
        assert_eq!(l[0], 3);
    }
}

//! Pure, host-testable preprocessing for a staged B/W update.
//!
//! Three deterministic transforms over caller-provided slices:
//!
//! - [`delta_has_red`] — decides whether the red stage may be omitted, by
//!   testing the dirty bitmap against the red-target plane.
//! - [`pack_content_stage`] — fused: computes each pixel's action and writes
//!   the two RAM planes inline, without an intermediate action array.
//! - [`pack_correction_stage`] — fused correction-stage plane builder.
//!
//! Both live in the crate (rather than the firmware) so they compile and run
//! under `cargo test` on the host target, where the firmware's nRF-only modules
//! cannot link.  The firmware re-exports thin wrappers.

use super::{classify, class_in_stage, ledger_apply_stage, Class, StageAction};

/// True if any pixel in the dirty set targets red — i.e. the dirty bitmap and
/// the red-target plane share a set bit. Both are 1bpp MSB-first, row-major,
/// same geometry. Used to decide red-stage omission.
///
/// # Arguments
///
/// * `dirty` - dirty bitmap, 1bpp MSB-first row-major
/// * `red_target` - desired red plane, same geometry as `dirty`
///
/// # Returns
///
/// `true` if at least one dirty pixel targets red.
///
/// # Examples
///
/// ```ignore
/// assert!(delta_has_red(&[0x0F], &[0x08])); // overlap in the low nibble
/// assert!(!delta_has_red(&[0xF0], &[0x0F])); // red set only outside dirty
/// ```
pub fn delta_has_red(dirty: &[u8], red_target: &[u8]) -> bool {
    dirty.iter().zip(red_target).any(|(&d, &r)| d & r != 0)
}

/// Read bit `(x, y)` from a 1bpp MSB-first, row-major plane.
#[inline]
fn bit(plane: &[u8], stride: usize, x: usize, y: usize) -> bool {
    (plane[y * stride + x / 8] >> (7 - (x % 8))) & 1 == 1
}

/// Write a pixel's `(red, bw)` bits into both RAM planes at position `(x, y)`.
///
/// Both planes are pre-cleared to 0 by the caller; only set bits need writing.
#[inline]
fn write_bits(
    bw_plane: &mut [u8],
    red_plane: &mut [u8],
    stride: usize,
    x: usize,
    y: usize,
    red: bool,
    bw: bool,
) {
    let byte = y * stride + x / 8;
    let mask = 0x80u8 >> (x % 8);
    if bw {
        bw_plane[byte] |= mask;
    }
    if red {
        red_plane[byte] |= mask;
    }
}

/// Build the two RAM planes (1bpp MSB-first) for content `stage`, computing
/// each pixel's action inline — no intermediate action array.
///
/// Clears both planes first. Per pixel: not-dirty OR class not in this stage →
/// NoOp; else DriveWhite/DriveBlack by `bw_target` bit. When `ledger` is
/// `Some`, books the drive via `ledger_apply_stage(+1 white / -1 black)`;
/// when `None`, skips booking entirely (same plane output, no allocation needed).
///
/// NoOp pixels set BOTH plane bits (`red=1, bw=1`), matching `StageAction::NoOp`
/// semantics.
///
/// # Arguments
///
/// * `stage` - 0-based stage index within the update
/// * `w` - panel width in pixels
/// * `h` - panel height in pixels
/// * `bw_target` - desired B/W plane, 1bpp MSB-first row-major (bit set ⇒ white)
/// * `dirty` - dirty bitmap, 1bpp MSB-first row-major
/// * `dist` - per-pixel distance map (len `w*h`) from `distance_transform`
/// * `ledger` - optional per-pixel signed charge ledger (len `w*h`), updated in
///   place when `Some`; pass `None` to omit DC-balance bookkeeping
/// * `bw_plane` - output B/W RAM plane (len `w.div_ceil(8) * h`), cleared then filled
/// * `red_plane` - output red RAM plane (len `w.div_ceil(8) * h`), cleared then filled
pub fn pack_content_stage(
    stage: usize,
    w: usize,
    h: usize,
    bw_target: &[u8],
    dirty: &[u8],
    dist: &[u8],
    ledger: Option<&mut [i8]>,
    bw_plane: &mut [u8],
    red_plane: &mut [u8],
) {
    bw_plane.fill(0);
    red_plane.fill(0);
    let stride = w.div_ceil(8);
    // Reborrow ledger as a plain slice reference to avoid partial-move issues in
    // the loop; `Option<&mut T>` reborrow trick lets us pass `None` or `Some`.
    let ledger_ref: Option<&mut [i8]> = ledger;
    // Split into a raw pointer + present flag so the borrow checker is happy
    // across the loop body.  Safety: we access only index `i` which is unique
    // per iteration.
    let (ledger_ptr, ledger_present) = match ledger_ref {
        Some(l) => (l.as_mut_ptr(), true),
        None => (core::ptr::null_mut(), false),
    };
    for i in 0..w * h {
        let x = i % w;
        let y = i / w;
        let action = if !bit(dirty, stride, x, y) || !class_in_stage(classify(dist, w, h, i), stage) {
            StageAction::NoOp
        } else {
            let to_white = bit(bw_target, stride, x, y);
            if ledger_present {
                // SAFETY: `ledger_ptr` is non-null (ledger_present guards it),
                // the slice length is `w*h`, and `i < w*h`.
                let ledger_slice = unsafe { core::slice::from_raw_parts_mut(ledger_ptr, w * h) };
                ledger_apply_stage(ledger_slice, i, if to_white { 1 } else { -1 });
            }
            if to_white { StageAction::DriveWhite } else { StageAction::DriveBlack }
        };
        let (red, bw) = action.ram_bits();
        write_bits(bw_plane, red_plane, stride, x, y, red, bw);
    }
}

/// White-only isolated boost stage: drive ONLY pixels that are dirty,
/// classified [`Class::Isolated`], and whose target is white.
///
/// Panel physics: a lone white pixel against a black surround is under-driven
/// by the colour-symmetric content stages — fringing fields from the black
/// neighbours pull it dark — so it gets one extra white pulse that its
/// black-on-white counterpart does not need.  All other pixels are `NoOp`.
///
/// Clears both planes first.
///
/// # Arguments
///
/// * `w`, `h` - panel dimensions in pixels
/// * `bw_target` - desired B/W frame, 1bpp MSB-first row-major (1 = white)
/// * `dirty` - dirty bitmap, same geometry
/// * `dist` - packed 2bpp distance map from [`super::distance_transform`]
/// * `bw_plane`, `red_plane` - output RAM planes, 1bpp MSB-first row-major
///
/// # Returns
///
/// The number of pixels driven, so the caller can skip the stage activation
/// entirely when no isolated-white pixel qualifies (saving one ~80 ms panel
/// activation).
pub fn pack_white_boost_stage(
    w: usize,
    h: usize,
    bw_target: &[u8],
    dirty: &[u8],
    dist: &[u8],
    bw_plane: &mut [u8],
    red_plane: &mut [u8],
) -> usize {
    bw_plane.fill(0);
    red_plane.fill(0);
    let stride = w.div_ceil(8);
    let mut driven = 0usize;
    for i in 0..w * h {
        let x = i % w;
        let y = i / w;
        let action = if bit(dirty, stride, x, y)
            && bit(bw_target, stride, x, y)
            && matches!(classify(dist, w, h, i), Class::Isolated)
        {
            driven += 1;
            StageAction::DriveWhite
        } else {
            StageAction::NoOp
        };
        let (red, bw) = action.ram_bits();
        write_bits(bw_plane, red_plane, stride, x, y, red, bw);
    }
    driven
}

/// Count cardinal (4-way) neighbours of pixel `(x, y)` in `target` that share its
/// target colour (1bpp MSB-first, row-major).
///
/// Off-grid neighbours are NOT counted, so an edge pixel can have at most 3
/// — it is never "fully interior" and receives a little extra drive (panel edges
/// suffer the worst fringing fields).
///
/// # Arguments
///
/// * `target` - desired B/W plane, 1bpp MSB-first row-major (bit=1 ⇒ white)
/// * `stride` - row stride in bytes (`w.div_ceil(8)`)
/// * `w`, `h` - panel dimensions in pixels
/// * `x`, `y` - pixel coordinates (must be in-bounds)
///
/// # Returns
///
/// Count of in-bounds cardinal neighbours whose target bit matches `(x, y)`.
fn same_colour_neighbours(target: &[u8], stride: usize, w: usize, h: usize, x: usize, y: usize) -> u8 {
    let c = bit(target, stride, x, y);
    let mut n = 0u8;
    if x > 0 && bit(target, stride, x - 1, y) == c { n += 1; }
    if x + 1 < w && bit(target, stride, x + 1, y) == c { n += 1; }
    if y > 0 && bit(target, stride, x, y - 1) == c { n += 1; }
    if y + 1 < h && bit(target, stride, x, y + 1) == c { n += 1; }
    n
}

/// Build the two RAM planes for erosion `stage` (0-based).
///
/// A dirty pixel is driven iff its same-colour neighbour count is `<= (4 - stage)`
/// (saturating).  Stage 0 drives every dirty pixel; each later stage excludes
/// the pixels with the most same-colour neighbours, so isolated pixels accumulate
/// the most drive pulses.
///
/// `DriveWhite` where the target bit is set (white target); `DriveBlack` otherwise.
/// All non-driven pixels (outside dirty set, or above the neighbour limit) are
/// `NoOp` (`red=1, bw=1`).  Both planes are cleared first.
///
/// Returns the number of pixels driven so the caller can stop once a stage drives
/// nothing (the driven set only shrinks monotonically with stage).
///
/// # Arguments
///
/// * `stage` - 0-based erosion stage index
/// * `w`, `h` - panel dimensions in pixels
/// * `bw_target` - desired B/W plane, 1bpp MSB-first row-major (bit=1 ⇒ white)
/// * `dirty` - dirty bitmap, same geometry as `bw_target`
/// * `bw_plane` - output B/W RAM plane (len `w.div_ceil(8) * h`), cleared then filled
/// * `red_plane` - output red RAM plane (len `w.div_ceil(8) * h`), cleared then filled
///
/// # Returns
///
/// The number of pixels driven in this stage.
pub fn pack_erosion_stage(
    stage: usize,
    w: usize,
    h: usize,
    bw_target: &[u8],
    dirty: &[u8],
    bw_plane: &mut [u8],
    red_plane: &mut [u8],
) -> usize {
    bw_plane.fill(0);
    red_plane.fill(0);
    let stride = w.div_ceil(8);
    let limit = 4u8.saturating_sub(stage as u8);
    let mut driven = 0usize;
    for i in 0..w * h {
        let (x, y) = (i % w, i / w);
        let action = if bit(dirty, stride, x, y)
            && same_colour_neighbours(bw_target, stride, w, h, x, y) <= limit
        {
            driven += 1;
            if bit(bw_target, stride, x, y) { StageAction::DriveWhite } else { StageAction::DriveBlack }
        } else {
            StageAction::NoOp
        };
        let (red, bw) = action.ram_bits();
        write_bits(bw_plane, red_plane, stride, x, y, red, bw);
    }
    driven
}

/// Build the two RAM planes for the correction stage from `corr_mask`, booking
/// the step toward zero. Per pixel: flagged & ledger>0 → DriveBlack (book -1);
/// flagged & ledger<0 → DriveWhite (book +1); else NoOp.
///
/// Clears both planes first. NoOp pixels set BOTH plane bits.
///
/// # Arguments
///
/// * `w` - panel width in pixels
/// * `h` - panel height in pixels
/// * `corr_mask` - correction bitmap from [`mark_corrections`], 1bpp MSB-first
///   row-major (len `w.div_ceil(8) * h`)
/// * `ledger` - per-pixel signed charge ledger (len `w*h`), updated in place
/// * `bw_plane` - output B/W RAM plane (len `w.div_ceil(8) * h`), cleared then filled
/// * `red_plane` - output red RAM plane (len `w.div_ceil(8) * h`), cleared then filled
pub fn pack_correction_stage(
    w: usize,
    h: usize,
    corr_mask: &[u8],
    ledger: &mut [i8],
    bw_plane: &mut [u8],
    red_plane: &mut [u8],
) {
    bw_plane.fill(0);
    red_plane.fill(0);
    let stride = w.div_ceil(8);
    for i in 0..w * h {
        let (x, y) = (i % w, i / w);
        let action = if !bit(corr_mask, stride, x, y) {
            StageAction::NoOp
        } else {
            match ledger[i].signum() {
                1 => {
                    ledger_apply_stage(ledger, i, -1);
                    StageAction::DriveBlack
                }
                -1 => {
                    ledger_apply_stage(ledger, i, 1);
                    StageAction::DriveWhite
                }
                _ => StageAction::NoOp,
            }
        };
        let (red, bw) = action.ram_bits();
        write_bits(bw_plane, red_plane, stride, x, y, red, bw);
    }
}

/// For each dirty pixel, set its bit in `corr_mask` (1bpp MSB-first, row-major,
/// len = `w.div_ceil(8) * h`) iff its carried-in imbalance exceeds `threshold`
/// in magnitude.  MUST be called at update start, before [`pack_content_stage`]
/// mutates the ledger, so the threshold test sees the pre-update imbalance.
///
/// A pixel is flagged iff its `dirty` bit is set AND
/// `ledger[i].unsigned_abs() > threshold as u8`.  [`i8::unsigned_abs`] handles
/// [`i8::MIN`] without overflow (it returns `128u8`).  `corr_mask` is cleared
/// first.
///
/// # Arguments
///
/// * `dirty` - dirty bitmap, 1bpp MSB-first row-major
/// * `ledger` - per-pixel signed charge ledger (len `w*h`)
/// * `threshold` - imbalance magnitude above which a pixel is flagged
/// * `w` - panel width in pixels
/// * `h` - panel height in pixels
/// * `corr_mask` - output correction bitmap (len `w.div_ceil(8) * h`), cleared
///   then filled in place
///
/// # Returns
///
/// The number of pixels flagged, so the caller can skip the correction stage
/// when it is zero.
pub fn mark_corrections(
    dirty: &[u8],
    ledger: &[i8],
    threshold: i8,
    w: usize,
    h: usize,
    corr_mask: &mut [u8],
) -> usize {
    corr_mask.iter_mut().for_each(|b| *b = 0);
    let stride = w.div_ceil(8);
    let thresh = threshold.unsigned_abs();
    let mut count = 0;
    for i in 0..w * h {
        let (x, y) = (i % w, i / w);
        if bit(dirty, stride, x, y) && ledger[i].unsigned_abs() > thresh {
            corr_mask[y * stride + x / 8] |= 0x80 >> (x % 8);
            count += 1;
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::super::{distance_transform, Class};
    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// Pack a row-major bool grid into a 1bpp MSB-first plane.
    fn pack(grid: &[&[u8]]) -> (Vec<u8>, usize, usize) {
        let h = grid.len();
        let w = grid[0].len();
        let stride = w.div_ceil(8);
        let mut buf = vec![0u8; stride * h];
        for (y, row) in grid.iter().enumerate() {
            for (x, &v) in row.iter().enumerate() {
                if v != 0 {
                    buf[y * stride + x / 8] |= 0x80 >> (x % 8);
                }
            }
        }
        (buf, w, h)
    }

    /// Read a single bit from a 1bpp MSB-first plane.
    fn plane_bit(plane: &[u8], stride: usize, x: usize, y: usize) -> bool {
        (plane[y * stride + x / 8] >> (7 - (x % 8))) & 1 == 1
    }

    /// Read a single bit from a 1bpp MSB-first plane.
    fn mask_bit(plane: &[u8], stride: usize, x: usize, y: usize) -> bool {
        plane_bit(plane, stride, x, y)
    }

    #[test]
    fn delta_has_red_overlap_true() {
        // dirty low nibble overlaps a red bit at x=4.
        assert!(delta_has_red(&[0x0F], &[0x08]));
    }

    #[test]
    fn delta_has_red_outside_dirty_false() {
        // red set only where dirty is clear → no overlap.
        assert!(!delta_has_red(&[0xF0, 0x00], &[0x00, 0xFF]));
    }

    #[test]
    fn delta_has_red_all_zero_false() {
        assert!(!delta_has_red(&[0x00, 0x00], &[0x00, 0x00]));
    }

    /// 5x5 fixture: a lone pixel at (0,0) (Isolated) and a 3x3 solid cluster in
    /// the lower-right (center Interior, faces Edge).  All pixels are dirty; the
    /// B/W target marks the lone pixel white, the cluster black.
    fn fixture() -> (Vec<u8>, Vec<u8>, Vec<u8>, usize, usize) {
        let target_grid: [&[u8]; 5] = [
            &[1, 0, 0, 0, 0],
            &[0, 0, 0, 0, 0],
            &[0, 0, 1, 1, 1],
            &[0, 0, 1, 1, 1],
            &[0, 0, 1, 1, 1],
        ];
        let (bw_target, w, h) = pack(&target_grid);
        // Everything dirty.
        let dirty_grid: [&[u8]; 5] = [&[1; 5]; 5];
        let (dirty, _, _) = pack(&dirty_grid);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);
        (bw_target, dirty, dist, w, h)
    }

    fn class_at(dist: &[u8], w: usize, h: usize, x: usize, y: usize) -> Class {
        classify(dist, w, h, y * w + x)
    }

    #[test]
    fn fixture_classes_are_as_expected() {
        let (_, _, dist, w, h) = fixture();
        assert_eq!(class_at(&dist, w, h, 0, 0), Class::Isolated, "lone pixel");
        // Cluster center of the 3x3 (3,3) sits one row/col in from each face →
        // distance 2, not a local max → Edge (a 3x3 has no distance-3 interior).
        assert_eq!(class_at(&dist, w, h, 3, 3), Class::Edge, "cluster center");
    }

    #[test]
    fn non_dirty_pixels_are_noop_every_stage() {
        let (bw_target, _, dist, w, h) = fixture();
        // No pixel dirty.
        let dirty = vec![0u8; w.div_ceil(8) * h];
        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        for stage in 0..3 {
            pack_content_stage(
                stage, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
                &mut bw_plane, &mut red_plane,
            );
            // Non-dirty pixels → NoOp → BOTH plane bits set.
            for y in 0..h {
                for x in 0..w {
                    assert!(
                        plane_bit(&bw_plane, stride, x, y),
                        "stage {stage} pixel ({x},{y}) bw must be set (NoOp)"
                    );
                    assert!(
                        plane_bit(&red_plane, stride, x, y),
                        "stage {stage} pixel ({x},{y}) red must be set (NoOp)"
                    );
                }
            }
            assert!(ledger.iter().all(|&l| l == 0), "ledger untouched stage {stage}");
        }
    }

    #[test]
    fn stage0_drives_all_dirty_per_target() {
        let (bw_target, dirty, dist, w, h) = fixture();
        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        pack_content_stage(
            0, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
            &mut bw_plane, &mut red_plane,
        );
        for y in 0..h {
            for x in 0..w {
                let to_white = plane_bit(&bw_target, stride, x, y);
                if to_white {
                    // DriveWhite: bw=1, red=0
                    assert!(plane_bit(&bw_plane, stride, x, y), "DriveWhite bw at ({x},{y})");
                    assert!(!plane_bit(&red_plane, stride, x, y), "DriveWhite red=0 at ({x},{y})");
                } else {
                    // DriveBlack: bw=0, red=0
                    assert!(!plane_bit(&bw_plane, stride, x, y), "DriveBlack bw=0 at ({x},{y})");
                    assert!(!plane_bit(&red_plane, stride, x, y), "DriveBlack red=0 at ({x},{y})");
                }
            }
        }
    }

    #[test]
    fn stage1_drives_edge_and_isolated_only() {
        let (bw_target, dirty, dist, w, h) = fixture();
        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        pack_content_stage(
            1, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
            &mut bw_plane, &mut red_plane,
        );
        for i in 0..w * h {
            let x = i % w;
            let y = i / w;
            // NoOp = both bits set; driven = at least one clear
            let is_noop = plane_bit(&bw_plane, stride, x, y) && plane_bit(&red_plane, stride, x, y);
            let class = classify(&dist, w, h, i);
            let expect_driven = matches!(class, Class::Edge | Class::Isolated);
            assert_eq!(!is_noop, expect_driven, "pixel {i} class {class:?}");
        }
    }

    #[test]
    fn stage2_drives_isolated_only() {
        let (bw_target, dirty, dist, w, h) = fixture();
        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        pack_content_stage(
            2, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
            &mut bw_plane, &mut red_plane,
        );
        for i in 0..w * h {
            let x = i % w;
            let y = i / w;
            let is_noop = plane_bit(&bw_plane, stride, x, y) && plane_bit(&red_plane, stride, x, y);
            let class = classify(&dist, w, h, i);
            let expect_driven = matches!(class, Class::Isolated);
            assert_eq!(!is_noop, expect_driven, "pixel {i} class {class:?}");
        }
    }

    #[test]
    fn isolated_accumulates_magnitude_three_interior_one() {
        // Build a fixture with both a genuine Interior pixel (7x7 white block,
        // so its center has distance ≥3) and an Isolated lone white pixel, so
        // cumulative ledger magnitudes can be checked.  Target bit set ⇒ white,
        // so every driven pixel here is DriveWhite (positive sign).
        let mut target_grid: Vec<Vec<u8>> = (0..9).map(|_| vec![0u8; 9]).collect();
        target_grid[0][0] = 1; // lone white pixel at (0,0)
        for row in target_grid.iter_mut().take(9).skip(2) {
            for cell in row.iter_mut().take(9).skip(2) {
                *cell = 1; // 7x7 white block in the lower-right
            }
        }
        let rows: Vec<&[u8]> = target_grid.iter().map(|r| r.as_slice()).collect();
        let (bw_target, w, h) = pack(&rows);
        let dirty_grid: Vec<Vec<u8>> = (0..9).map(|_| vec![1u8; 9]).collect();
        let drows: Vec<&[u8]> = dirty_grid.iter().map(|r| r.as_slice()).collect();
        let (dirty, _, _) = pack(&drows);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);

        // Confirm we actually have an Interior pixel at the block center (5,5).
        let interior_i = 5 * w + 5;
        assert_eq!(classify(&dist, w, h, interior_i), Class::Interior, "block center");
        let isolated_i = 0; // (0,0)
        assert_eq!(classify(&dist, w, h, isolated_i), Class::Isolated, "lone pixel");

        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        for stage in 0..3 {
            pack_content_stage(
                stage, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
                &mut bw_plane, &mut red_plane,
            );
        }
        // Isolated driven in all 3 stages → magnitude 3; both targets white → +.
        assert_eq!(ledger[isolated_i], 3, "isolated +3");
        // Interior driven only stage 0 → magnitude 1.
        assert_eq!(ledger[interior_i], 1, "interior +1");
    }

    #[test]
    fn ledger_sign_white_positive_black_negative() {
        // 1x2 row: pixel 0 → white (+), pixel 1 → black (-); both Isolated/Edge.
        let target_grid: [&[u8]; 1] = [&[1, 0]];
        let (bw_target, w, h) = pack(&target_grid);
        let dirty_grid: [&[u8]; 1] = [&[1, 1]];
        let (dirty, _, _) = pack(&dirty_grid);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);
        let mut ledger = vec![0i8; w * h];
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        pack_content_stage(
            0, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
            &mut bw_plane, &mut red_plane,
        );
        assert!(ledger[0] > 0, "to-white positive");
        assert!(ledger[1] < 0, "to-black negative");
    }

    #[test]
    fn mark_corrections_flags_above_threshold_dirty_only() {
        // 1x3 row, all classified the same; only the ledger/dirty differ.
        let w: usize = 3;
        let h: usize = 1;
        let stride = w.div_ceil(8);
        // Pixel 0: +7, dirty → flagged. Pixel 1: +6, dirty → NOT (not > 6).
        // Pixel 2: +7 but NOT dirty → not flagged.
        let dirty_grid: [&[u8]; 1] = [&[1, 1, 0]];
        let (dirty, _, _) = pack(&dirty_grid);
        let ledger = [7i8, 6, 7];
        let mut corr_mask = vec![0xFFu8; stride * h]; // pre-dirtied; must be cleared
        let count = mark_corrections(&dirty, &ledger, 6, w, h, &mut corr_mask);
        assert_eq!(count, 1, "only pixel 0 qualifies");
        assert!(mask_bit(&corr_mask, stride, 0, 0), "pixel 0 flagged");
        assert!(!mask_bit(&corr_mask, stride, 1, 0), "pixel 1 at threshold not flagged");
        assert!(!mask_bit(&corr_mask, stride, 2, 0), "pixel 2 not dirty");
    }

    #[test]
    fn mark_corrections_handles_negative_imbalance() {
        let w: usize = 2;
        let h: usize = 1;
        let stride = w.div_ceil(8);
        let dirty_grid: [&[u8]; 1] = [&[1, 1]];
        let (dirty, _, _) = pack(&dirty_grid);
        let ledger = [-7i8, -6];
        let mut corr_mask = vec![0u8; stride * h];
        let count = mark_corrections(&dirty, &ledger, 6, w, h, &mut corr_mask);
        assert_eq!(count, 1, "only -7 exceeds magnitude 6");
        assert!(mask_bit(&corr_mask, stride, 0, 0), "-7 flagged");
        assert!(!mask_bit(&corr_mask, stride, 1, 0), "-6 at threshold not flagged");
    }

    #[test]
    fn mark_corrections_i8_min_does_not_overflow() {
        // i8::MIN (-128) → unsigned_abs() == 128, well above any i8 threshold.
        let w: usize = 1;
        let h: usize = 1;
        let stride = w.div_ceil(8);
        let dirty_grid: [&[u8]; 1] = [&[1]];
        let (dirty, _, _) = pack(&dirty_grid);
        let ledger = [i8::MIN];
        let mut corr_mask = vec![0u8; stride * h];
        let count = mark_corrections(&dirty, &ledger, 6, w, h, &mut corr_mask);
        assert_eq!(count, 1, "i8::MIN exceeds threshold without overflow");
        assert!(mask_bit(&corr_mask, stride, 0, 0));
    }

    #[test]
    fn pack_correction_stage_drives_toward_neutral_and_books_step() {
        // 1x4 row: +7 (too white), -7 (too black), 0 (safety), unflagged.
        let w: usize = 4;
        let h: usize = 1;
        let stride = w.div_ceil(8);
        // Flag pixels 0, 1, 2; leave 3 unflagged.
        let mut corr_mask = vec![0u8; stride * h];
        for x in 0..3 {
            corr_mask[x / 8] |= 0x80 >> (x % 8);
        }
        let mut ledger = [7i8, -7, 0, 7];
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];
        pack_correction_stage(w, h, &corr_mask, &mut ledger, &mut bw_plane, &mut red_plane);

        // Pixel 0: too white (>0) → DriveBlack: bw=0, red=0; ledger 7→6.
        assert!(!plane_bit(&bw_plane, stride, 0, 0), "too white bw=0 (DriveBlack)");
        assert!(!plane_bit(&red_plane, stride, 0, 0), "too white red=0 (DriveBlack)");
        assert_eq!(ledger[0], 6, "+7 steps to +6");

        // Pixel 1: too black (<0) → DriveWhite: bw=1, red=0; ledger -7→-6.
        assert!(plane_bit(&bw_plane, stride, 1, 0), "too black bw=1 (DriveWhite)");
        assert!(!plane_bit(&red_plane, stride, 1, 0), "too black red=0 (DriveWhite)");
        assert_eq!(ledger[1], -6, "-7 steps to -6");

        // Pixel 2: flagged but ledger 0 → NoOp: both bits set.
        assert!(plane_bit(&bw_plane, stride, 2, 0), "ledger 0 flagged bw=1 (NoOp)");
        assert!(plane_bit(&red_plane, stride, 2, 0), "ledger 0 flagged red=1 (NoOp)");
        assert_eq!(ledger[2], 0, "ledger 0 untouched");

        // Pixel 3: unflagged → NoOp: both bits set.
        assert!(plane_bit(&bw_plane, stride, 3, 0), "unflagged bw=1 (NoOp)");
        assert!(plane_bit(&red_plane, stride, 3, 0), "unflagged red=1 (NoOp)");
        assert_eq!(ledger[3], 7, "unflagged ledger untouched");
    }

    #[test]
    fn pack_content_stage_none_ledger_produces_same_planes() {
        // Verify that `None` ledger yields identical plane output to `Some` and
        // that a sentinel ledger is not touched.
        let (bw_target, dirty, dist, w, h) = fixture();
        let stride = w.div_ceil(8);
        let mut bw_some = vec![0u8; stride * h];
        let mut red_some = vec![0u8; stride * h];
        let mut bw_none = vec![0u8; stride * h];
        let mut red_none = vec![0u8; stride * h];
        let mut ledger_some = vec![0i8; w * h];
        // Sentinel ledger that must not be modified.
        let sentinel: Vec<i8> = vec![42i8; w * h];

        // Run both variants for all stages and compare output planes.
        for stage in 0..3 {
            pack_content_stage(
                stage, w, h, &bw_target, &dirty, &dist, Some(&mut ledger_some),
                &mut bw_some, &mut red_some,
            );
            pack_content_stage(
                stage, w, h, &bw_target, &dirty, &dist, None,
                &mut bw_none, &mut red_none,
            );
            assert_eq!(bw_some, bw_none, "stage {stage} bw planes must match");
            assert_eq!(red_some, red_none, "stage {stage} red planes must match");
        }
        // The sentinel slice was never passed in, so it must be untouched.
        assert!(sentinel.iter().all(|&v| v == 42), "sentinel unchanged");
    }

    #[test]
    fn end_to_end_correction_books_one_step_from_carried_in() {
        // Isolated lone white pixel at (0,0); carried-in ledger +7 (above THRESH 6).
        // mark_corrections (pre-stages) must flag it; content stages add +3 toward
        // white (driven in all 3); correction stage then books one step toward 0.
        let target_grid: [&[u8]; 1] = [&[1]];
        let (bw_target, w, h) = pack(&target_grid);
        let dirty_grid: [&[u8]; 1] = [&[1]];
        let (dirty, _, _) = pack(&dirty_grid);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);
        assert_eq!(classify(&dist, w, h, 0), Class::Isolated, "lone pixel isolated");

        let stride = w.div_ceil(8);
        let mut ledger = vec![7i8; w * h];
        let mut corr_mask = vec![0u8; stride * h];
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];

        // Snapshot BEFORE any stage mutates the ledger.
        let flagged = mark_corrections(&dirty, &ledger, 6, w, h, &mut corr_mask);
        assert_eq!(flagged, 1, "carried-in +7 flags the pixel");

        // Content stages 0..2: isolated driven in all three → +3.
        for stage in 0..3 {
            pack_content_stage(
                stage, w, h, &bw_target, &dirty, &dist, Some(&mut ledger),
                &mut bw_plane, &mut red_plane,
            );
        }
        assert_eq!(ledger[0], 10, "carried-in 7 + 3 content stages");

        // Correction stage: too white (>0) → DriveBlack, books one step toward 0.
        pack_correction_stage(w, h, &corr_mask, &mut ledger, &mut bw_plane, &mut red_plane);
        // DriveBlack: bw=0, red=0
        assert!(!plane_bit(&bw_plane, stride, 0, 0), "driven toward neutral: DriveBlack bw=0");
        assert!(!plane_bit(&red_plane, stride, 0, 0), "driven toward neutral: DriveBlack red=0");
        assert_eq!(ledger[0], 9, "exactly one step back toward zero");
    }

    #[test]
    fn white_boost_drives_only_isolated_white() {
        // 7x7 all-black field with a single white pixel at the centre (3,3):
        // that pixel is the only Isolated-AND-white pixel → driven once.
        let mut target_grid_rows: Vec<Vec<u8>> = vec![vec![0u8; 7]; 7];
        target_grid_rows[3][3] = 1;
        let target_grid: Vec<&[u8]> = target_grid_rows.iter().map(|r| r.as_slice()).collect();
        let (bw_target, w, h) = pack(&target_grid);
        let dirty_grid: [&[u8]; 7] = [&[1; 7]; 7];
        let (dirty, _, _) = pack(&dirty_grid);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];

        let driven =
            pack_white_boost_stage(w, h, &bw_target, &dirty, &dist, &mut bw_plane, &mut red_plane);

        // Only the lone white centre qualifies.
        assert_eq!(driven, 1);
        // Centre → DriveWhite: bw bit set, red clear.
        assert!(plane_bit(&bw_plane, stride, 3, 3), "isolated white driven: bw=1");
        assert!(!plane_bit(&red_plane, stride, 3, 3), "isolated white driven: red=0");
        // A black background pixel → NoOp: both bits set.
        assert!(plane_bit(&bw_plane, stride, 0, 0), "black pixel NoOp: bw=1");
        assert!(plane_bit(&red_plane, stride, 0, 0), "black pixel NoOp: red=1");
    }

    // ── Erosion stage tests ──────────────────────────────────────────────────

    /// Parse a grid string (rows separated by `|`, '1'=white, '0'=black) into a
    /// 1bpp MSB-first buffer.
    fn pack_str(grid: &str) -> (Vec<u8>, usize, usize) {
        let rows: Vec<&str> = grid.split('|').collect();
        let h = rows.len();
        let w = rows[0].len();
        let stride = w.div_ceil(8);
        let mut buf = vec![0u8; stride * h];
        for (y, row) in rows.iter().enumerate() {
            for (x, c) in row.chars().enumerate() {
                if c == '1' {
                    buf[y * stride + x / 8] |= 0x80 >> (x % 8);
                }
            }
        }
        (buf, w, h)
    }

    /// Helper: returns true if pixel `(x,y)` is in NoOp state (both bw and red
    /// bits set — the `(red=1, bw=1)` encoding for LUT3).
    fn is_noop_pixel(bw: &[u8], red: &[u8], stride: usize, x: usize, y: usize) -> bool {
        plane_bit(bw, stride, x, y) && plane_bit(red, stride, x, y)
    }

    /// 5×5 solid white block — all dirty.
    /// Centre (2,2) has 4 same-colour neighbours → driven only in stage 0 (limit=4).
    /// Top-edge non-corner (1,0) has 3 same-colour neighbours → driven in stages 0,1;
    /// NoOp in stage 2+ (limit=2 < 3).
    #[test]
    fn test_solid_white_erosion() {
        let (target, w, h) = pack_str("11111|11111|11111|11111|11111");
        let (dirty, _, _) = pack_str("11111|11111|11111|11111|11111");
        let stride = w.div_ceil(8);
        let plane_len = stride * h;
        let mut bw = vec![0u8; plane_len];
        let mut red = vec![0u8; plane_len];

        // Stage 0: drives all pixels (limit=4, all have <=4 same neighbours).
        let driven = pack_erosion_stage(0, w, h, &target, &dirty, &mut bw, &mut red);
        assert_eq!(driven, 25, "stage 0 should drive all 25 pixels");
        // Centre (2,2) driven as DriveWhite (target=1): bw=1, red=0.
        assert!(plane_bit(&bw, stride, 2, 2), "centre bw bit set stage 0");
        assert!(!plane_bit(&red, stride, 2, 2), "centre red bit clear stage 0 (DriveWhite)");

        // Stage 1: limit=3. Centre (2,2) has 4 same-colour neighbours → NoOp.
        let driven1 = pack_erosion_stage(1, w, h, &target, &dirty, &mut bw, &mut red);
        // NoOp encoding: bw=1, red=1 — check with helper
        assert!(is_noop_pixel(&bw, &red, stride, 2, 2), "centre is NoOp (not driven) in stage 1");
        // Top non-corner (1,0): right(2,0)=white, left(0,0)=white, down(1,1)=white → 3 neighbours ≤ 3 → driven
        assert!(!is_noop_pixel(&bw, &red, stride, 1, 0), "top-edge (1,0) driven in stage 1");
        assert!(plane_bit(&bw, stride, 1, 0), "(1,0) bw=1 DriveWhite stage 1");
        assert!(!plane_bit(&red, stride, 1, 0), "(1,0) red=0 DriveWhite stage 1");
        let _ = driven1; // count varies

        // Stage 2: limit=2. (1,0) has 3 same-colour neighbours → NoOp.
        pack_erosion_stage(2, w, h, &target, &dirty, &mut bw, &mut red);
        assert!(is_noop_pixel(&bw, &red, stride, 1, 0), "(1,0) is NoOp (not driven) in stage 2");
    }

    /// 5×5 all-black with one white centre pixel — all dirty.
    /// Centre (2,2) has 0 same-colour white neighbours → driven in ALL stages 0..4.
    /// At stage 4 (limit=0), only pixels with 0 same-colour neighbours are driven.
    #[test]
    fn test_lone_white_centre_erosion() {
        // all black (0) except centre (2,2) white (1)
        let (mut target, w, h) = pack_str("00000|00000|00000|00000|00000");
        let stride = w.div_ceil(8);
        target[2 * stride + 2 / 8] |= 0x80 >> (2 % 8);
        let (dirty, _, _) = pack_str("11111|11111|11111|11111|11111");
        let plane_len = stride * h;
        let mut bw = vec![0u8; plane_len];
        let mut red = vec![0u8; plane_len];

        for stage in 0..5 {
            let driven = pack_erosion_stage(stage, w, h, &target, &dirty, &mut bw, &mut red);
            // Centre (2,2) is white with 0 same-colour (white) neighbours → always driven.
            // DriveWhite: bw=1, red=0.
            assert!(!is_noop_pixel(&bw, &red, stride, 2, 2), "centre driven (not NoOp) in stage {stage}");
            assert!(plane_bit(&bw, stride, 2, 2), "centre bw=1 DriveWhite stage {stage}");
            assert!(!plane_bit(&red, stride, 2, 2), "centre red=0 DriveWhite stage {stage}");
            if stage == 4 {
                // limit=0: only pixels with 0 same-colour neighbours driven.
                // Adjacent black pixels (1,2),(3,2),(2,1),(2,3): each has 3 black same-colour
                // neighbours → NOT driven (NoOp: bw=1, red=1).
                assert_eq!(driven, 1, "stage 4 drives only the lone white centre");
                assert!(
                    is_noop_pixel(&bw, &red, stride, 1, 2),
                    "left of centre is NoOp (not driven) in stage 4"
                );
            }
        }
    }

    /// All-zero dirty buffer → driven == 0 for any stage.
    #[test]
    fn test_empty_dirty_drives_nothing() {
        let (target, w, h) = pack_str("11111|11111|11111|11111|11111");
        let dirty = vec![0u8; target.len()];
        let stride = w.div_ceil(8);
        let plane_len = stride * h;
        let mut bw = vec![0u8; plane_len];
        let mut red = vec![0u8; plane_len];
        for stage in 0..5 {
            let driven = pack_erosion_stage(stage, w, h, &target, &dirty, &mut bw, &mut red);
            assert_eq!(driven, 0, "empty dirty → 0 driven in stage {stage}");
        }
    }

    #[test]
    fn white_boost_skips_isolated_black() {
        // Lone BLACK pixel in a white field is Isolated but NOT white → the boost
        // must not drive it (its black-on-white counterpart needs no extra pulse).
        let mut target_grid_rows: Vec<Vec<u8>> = vec![vec![1u8; 7]; 7];
        target_grid_rows[3][3] = 0;
        let target_grid: Vec<&[u8]> = target_grid_rows.iter().map(|r| r.as_slice()).collect();
        let (bw_target, w, h) = pack(&target_grid);
        let dirty_grid: [&[u8]; 7] = [&[1; 7]; 7];
        let (dirty, _, _) = pack(&dirty_grid);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bw_target, w, h, &mut dist);
        let stride = w.div_ceil(8);
        let mut bw_plane = vec![0u8; stride * h];
        let mut red_plane = vec![0u8; stride * h];

        let _driven =
            pack_white_boost_stage(w, h, &bw_target, &dirty, &dist, &mut bw_plane, &mut red_plane);

        // The isolated pixel is black → NoOp (both bits set), never driven.
        assert!(plane_bit(&bw_plane, stride, 3, 3), "isolated black NoOp: bw=1");
        assert!(plane_bit(&red_plane, stride, 3, 3), "isolated black NoOp: red=1");
    }
}

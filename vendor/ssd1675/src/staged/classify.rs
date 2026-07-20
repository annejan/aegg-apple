//! Distance-to-boundary classification via a 2-pass chamfer distance transform.

/// Boundary class for a pixel; cumulative drive impulse decreases with distance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    /// Deep inside a same-colour cluster (distance >= 3): least extra impulse.
    Interior,
    /// Near a colour boundary (distance 1-2, not a local maximum).
    Edge,
    /// Lone / thin feature (distance-1 local maximum): most extra impulse.
    Isolated,
}

/// Read the 2-bit distance for pixel `i` from a packed map (4 px/byte, low bits first).
#[inline]
fn get_d(dist: &[u8], i: usize) -> u8 {
    (dist[i >> 2] >> ((i & 3) * 2)) & 0b11
}

/// Write a 2-bit distance (0..=3) for pixel `i` into a packed map.
#[inline]
fn set_d(dist: &mut [u8], i: usize, v: u8) {
    let shift = (i & 3) * 2;
    dist[i >> 2] = (dist[i >> 2] & !(0b11 << shift)) | ((v & 0b11) << shift);
}

/// Compute the chamfer (1,1) distance of every pixel to the nearest pixel of the
/// *opposite* colour, into `dist` (saturating at 3).
///
/// # Arguments
///
/// * `target` - 1bpp MSB-first, row-major bitmap; bit=1 is one colour, bit=0 the other.
/// * `width` - panel width in pixels.
/// * `height` - panel height in pixels.
/// * `dist` - packed output buffer of length `(width * height).div_ceil(4)`; each pixel
///   occupies 2 bits (4 pixels/byte, low bits first). `get_d(dist, i)` is the distance
///   for pixel `i` (row-major), clamped to `0..=3`. Off-grid neighbours count as the
///   opposite colour, so the panel edge is treated as a boundary.
pub fn distance_transform(target: &[u8], width: usize, height: usize, dist: &mut [u8]) {
    let stride = width.div_ceil(8);
    let get = |x: usize, y: usize| -> bool {
        let byte = y * stride + x / 8;
        (target[byte] >> (7 - (x % 8))) & 1 == 1
    };
    // Seed: distance 1 at boundary pixels (a neighbour of opposite colour), else "far" (3).
    for y in 0..height {
        for x in 0..width {
            let c = get(x, y);
            let mut boundary = false;
            // 4-neighbourhood; off-grid counts as opposite (panel edge is a boundary).
            let mut chk = |nx: isize, ny: isize| {
                if nx < 0 || ny < 0 || nx >= width as isize || ny >= height as isize {
                    boundary = true;
                } else if get(nx as usize, ny as usize) != c {
                    boundary = true;
                }
            };
            chk(x as isize - 1, y as isize);
            chk(x as isize + 1, y as isize);
            chk(x as isize, y as isize - 1);
            chk(x as isize, y as isize + 1);
            set_d(dist, y * width + x, if boundary { 1 } else { 3 });
        }
    }
    // Forward then backward chamfer relaxation (cap at 3).
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            let mut d = get_d(dist, i);
            if x > 0 {
                d = d.min(get_d(dist, i - 1).saturating_add(1));
            }
            if y > 0 {
                d = d.min(get_d(dist, i - width).saturating_add(1));
            }
            set_d(dist, i, d.min(3));
        }
    }
    for y in (0..height).rev() {
        for x in (0..width).rev() {
            let i = y * width + x;
            let mut d = get_d(dist, i);
            if x + 1 < width {
                d = d.min(get_d(dist, i + 1).saturating_add(1));
            }
            if y + 1 < height {
                d = d.min(get_d(dist, i + width).saturating_add(1));
            }
            set_d(dist, i, d.min(3));
        }
    }
}

/// Classify pixel `i` from its distance and a local-max test against `dist`.
///
/// # Arguments
///
/// * `dist` - packed distance buffer produced by [`distance_transform`] (2 bits/pixel,
///   `(width * height).div_ceil(4)` bytes, 4 pixels/byte low-bits-first).
/// * `width` - panel width in pixels.
/// * `height` - panel height in pixels.
/// * `i` - row-major index of the pixel to classify.
///
/// # Returns
///
/// [`Class::Interior`] when the pixel's distance >= 3, [`Class::Isolated`] when its
/// distance == 1 and it is a local maximum among its 4-neighbours, otherwise [`Class::Edge`].
pub fn classify(dist: &[u8], width: usize, height: usize, i: usize) -> Class {
    let d = get_d(dist, i);
    if d >= 3 {
        return Class::Interior;
    }
    let x = i % width;
    let y = i / width;
    // Local maximum among 4-neighbours => isolated/thin feature.
    let mut is_local_max = true;
    let mut chk = |nx: usize, ny: usize| {
        if get_d(dist, ny * width + nx) > d {
            is_local_max = false;
        }
    };
    if x > 0 {
        chk(x - 1, y);
    }
    if x + 1 < width {
        chk(x + 1, y);
    }
    if y > 0 {
        chk(x, y - 1);
    }
    if y + 1 < height {
        chk(x, y + 1);
    }
    if d == 1 && is_local_max {
        Class::Isolated
    } else {
        Class::Edge
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// Build a 1bpp MSB-first bitmap from a row-major grid of `0`/`1` cells.
    fn bitmap(grid: &[&[u8]]) -> (Vec<u8>, usize, usize) {
        let height = grid.len();
        let width = grid[0].len();
        let stride = width.div_ceil(8);
        let mut buf = vec![0u8; stride * height];
        for (y, row) in grid.iter().enumerate() {
            for (x, &v) in row.iter().enumerate() {
                if v != 0 {
                    buf[y * stride + x / 8] |= 0x80 >> (x % 8);
                }
            }
        }
        (buf, width, height)
    }

    #[test]
    fn lone_pixel_is_isolated() {
        // 5x5 all 0 with a single 1 in the centre -> that 1 is isolated.
        let g: Vec<Vec<u8>> = (0..5)
            .map(|y| (0..5).map(|x| if x == 2 && y == 2 { 1 } else { 0 }).collect())
            .collect();
        let rows: Vec<&[u8]> = g.iter().map(|r| r.as_slice()).collect();
        let (bm, w, h) = bitmap(&rows);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bm, w, h, &mut dist);
        let c = classify(&dist, w, h, 2 * w + 2);
        assert_eq!(c, Class::Isolated);
    }

    #[test]
    fn cluster_interior_is_interior() {
        // 7x7 solid block of 1s -> centre pixel has distance >=3 -> Interior.
        let g: Vec<Vec<u8>> = (0..7).map(|_| (0..7).map(|_| 1u8).collect()).collect();
        let rows: Vec<&[u8]> = g.iter().map(|r| r.as_slice()).collect();
        let (bm, w, h) = bitmap(&rows);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bm, w, h, &mut dist);
        assert_eq!(classify(&dist, w, h, 3 * w + 3), Class::Interior);
    }

    #[test]
    fn cluster_border_is_edge() {
        // 7x7 solid block -> a pixel on the block edge (not corner) is Edge.
        let g: Vec<Vec<u8>> = (0..7).map(|_| (0..7).map(|_| 1u8).collect()).collect();
        let rows: Vec<&[u8]> = g.iter().map(|r| r.as_slice()).collect();
        let (bm, w, h) = bitmap(&rows);
        let mut dist = vec![0u8; (w * h).div_ceil(4)];
        distance_transform(&bm, w, h, &mut dist);
        // (3,0) top edge, middle of the top row.
        assert_eq!(classify(&dist, w, h, 3), Class::Edge);
    }
}

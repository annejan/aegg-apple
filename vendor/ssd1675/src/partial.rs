//! Host-side differential update for SSD1675A tri-color (B/W/R) panels.
//!
//! Tracks committed panel state (`shadow`) + desired frame (`pending`)
//! + dirty bits per pixel.  Each `update_partial` drives only the
//! pixels marked dirty, in directions encoded by the LUT row a pixel's
//! `(RED, BW)` RAM bits select.
//!
//! Per-pixel encoding (drives the (RED, BW) RAM-plane bits at update
//! time — see Phase 4 plane builder):
//!
//! | RED | BW | LUT row | action            |
//! |-----|----|---------|-------------------|
//! |  0  |  0 | LUT0    | drive to BLACK    |
//! |  0  |  1 | LUT1    | drive to WHITE    |
//! |  1  |  0 | LUT2    | drive to RED      |
//! |  1  |  1 | LUT3    | IGNORE (no drive) |
//!
//! Unchanged pixels are encoded as (RED=1, BW=1) so the chip routes
//! them through LUT3, which `patch_lut_for_partial` has cleared to
//! all-zero bytes → no voltage applied → pixel held undisturbed.
//!
//! Other LUT mutation: kill the factory inversion / shake phases.
//! Factory LUT contains early phases where source polarity flips
//! across LUT0..LUT2 simultaneously to break particle ion-trapping;
//! visible as a panel-wide flicker on every refresh.  Acceptable on
//! full refresh, not on partial.  Heuristic in `patch_lut_for_partial`
//! zeros groups where LUT0..LUT2 phase bytes match (after LUT3 is
//! already zeroed by the same pass).

use crate::DisplayVariant;
use crate::graphics::Color;
use static_cell::ConstStaticCell;

/// Per-variant LUT body geometry used by the partial-update patches.
/// Both controllers in the SSD1675 family share row-major layout
/// (5 LUT rows × N phases) but differ in N and TP region position.
#[derive(Clone, Copy)]
pub struct Layout {
    /// Number of timing phases per LUT row.
    pub n_phases: usize,
    /// Bytes per LUT row in the waveform region (= n_phases).
    pub phases_per_row: usize,
    /// Start of the TP timing region (5 LUT rows × n_phases bytes).
    pub tp_base: usize,
    /// Bytes per TP entry (TP-A, TP-B, TP-C, TP-D, RP).
    pub tp_stride: usize,
    /// Total bytes in the LUT body the chip honours (waveform + TPs).
    /// Used as the upper bound for byte indexing.
    pub body_len: usize,
}

pub const SSD1675A_LAYOUT: Layout = Layout {
    n_phases: 7,
    phases_per_row: 7,
    tp_base: 35,
    tp_stride: 5,
    body_len: 70,
};

/// SSD1675B has 5 LUT rows × 10 phases (50 waveform bytes) followed by
/// 10 TPs × 5 bytes (50 bytes).  Last byte (TP9 RP) is unused per
/// `DisplayVariant::lut_byte_len()` returning 99.
pub const SSD1675B_LAYOUT: Layout = Layout {
    n_phases: 10,
    phases_per_row: 10,
    tp_base: 50,
    tp_stride: 5,
    body_len: 99,
};

impl Layout {
    /// Row-major byte offset for `(row, phase)` inside the waveform region.
    #[inline]
    pub const fn lut_byte(&self, row: usize, phase: usize) -> usize {
        row * self.phases_per_row + phase
    }
}

/// Map a `Color` to the `(RED bit, BW bit)` RAM-plane bits that
/// route the pixel to its drive LUT.
///
/// (RED=1, BW=1) is the IGNORE class — caller uses it for pixels
/// that should not be driven this refresh.  Never returned by this
/// function; reserved for the unchanged-pixel encoding.
pub fn color_to_ram_bits(c: Color) -> (bool, bool) {
    match c {
        Color::Black => (false, false), // → LUT0
        Color::White => (false, true),  // → LUT1
        Color::Red => (true, false),    // → LUT2
    }
}

/// How to handle phases where LUT0 == LUT1 == LUT2 are nonzero and
/// match — i.e., OTP inversion / shake phases or a uniform white-wipe
/// pre-drive baked into a pre-patched LUT.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvertHandling {
    /// Leave matched phases unchanged.  Use when the base LUT already
    /// has the inversion / wipe tuned correctly (e.g. the
    /// `patch_no_invert`-patched `no_invert` table — keeping the
    /// pre-wipe gives black-target pixels full contrast).
    Preserve,
    /// OTP shake refinement: preserve LUT0+LUT1 (B↔W still get
    /// ion-trap-break), zero LUT2 (red skips shake), halve
    /// TPxA/B/C/D and zero RPx.  Use on raw `select_full` OTP LUT.
    Refine,
    /// Zero LUT0/LUT1/LUT2 + TP for matched phases entirely.  Pixels
    /// drive straight to target with no pre-step.  Use when wipe
    /// leaves visible residue from prior interrupted refreshes.
    Kill,
}

// ────────────────────────────────────────────────────────────────────────
// Public LUT-patch entry points — dispatch per controller variant.
// All shared patch policy lives in the per-variant inner functions
// below so per-display tuning can diverge without bleeding into the
// other variant's path.
// ────────────────────────────────────────────────────────────────────────

/// Patch a factory LUT body for partial-update use.  Dispatches to
/// the per-variant patch policy.
pub fn patch_lut_for_partial(lut: &mut [u8], variant: DisplayVariant, invert: InvertHandling) {
    match variant {
        DisplayVariant::Ssd1675 => patch_partial_ssd1675a(lut, invert),
        DisplayVariant::Ssd1675B => patch_partial_ssd1675b(lut, invert),
    }
}

/// Drop red-drive cost from an already-`patch_lut_for_partial`-ed
/// LUT.  Use when no dirty pixel targets red (caller knows this from
/// `build_planes` returning `had_red = false`).  Must run after
/// `patch_lut_for_partial`.
pub fn patch_lut_skip_red(lut: &mut [u8], variant: DisplayVariant) {
    match variant {
        DisplayVariant::Ssd1675 => skip_red_ssd1675a(lut),
        DisplayVariant::Ssd1675B => skip_red_ssd1675b(lut),
    }
}

// ────────────────────────────────────────────────────────────────────────
// Shared layout-driven core — both variants currently use the same
// patch policy; only their byte geometry differs.  When tuning needs
// to diverge per controller, copy the body of this helper into the
// per-variant function below and edit it there.
// ────────────────────────────────────────────────────────────────────────

fn patch_partial_generic(lut: &mut [u8], layout: Layout, invert: InvertHandling) {
    if lut.len() < layout.body_len {
        return;
    }

    // Step 1: zero LUT3 (WW / ignore-class) phase bytes.  Pixels routed
    // to LUT3 via RAM bits (1,1) = unchanged in our encoding receive
    // no drive.
    for p in 0..layout.n_phases {
        lut[layout.lut_byte(3, p)] = 0;
    }

    // Step 2: zero LUT4 (VCOM).  Without this, OTP VCOM modulation
    // across phases applies to every pixel (including IGNORE-class
    // where source = GND from LUT3) → per-refresh DC bias that
    // accumulates → ink drift on untouched pixels.  Static VCOM (cmd
    // 0x2C trailer value) used instead.
    for p in 0..layout.n_phases {
        lut[layout.lut_byte(4, p)] = 0;
    }

    // Step 3: process matched LUT0==LUT1==LUT2 nonzero phases per
    // `invert` mode.
    if invert == InvertHandling::Preserve {
        return;
    }
    for phase in 0..layout.n_phases {
        let l0 = lut[layout.lut_byte(0, phase)];
        let l1 = lut[layout.lut_byte(1, phase)];
        let l2 = lut[layout.lut_byte(2, phase)];
        if l0 != 0 && l0 == l1 && l1 == l2 {
            match invert {
                InvertHandling::Preserve => {}
                InvertHandling::Kill => {
                    lut[layout.lut_byte(0, phase)] = 0;
                    lut[layout.lut_byte(1, phase)] = 0;
                    lut[layout.lut_byte(2, phase)] = 0;
                    let tp = layout.tp_base + phase * layout.tp_stride;
                    let end = (tp + layout.tp_stride).min(lut.len());
                    lut[tp..end].fill(0);
                }
                InvertHandling::Refine => {
                    lut[layout.lut_byte(2, phase)] = 0;
                    let tp = layout.tp_base + phase * layout.tp_stride;
                    if tp + layout.tp_stride <= lut.len() {
                        lut[tp] /= 2; // TPxA
                        lut[tp + 1] /= 2; // TPxB
                        lut[tp + 2] /= 2; // TPxC
                        lut[tp + 3] /= 2; // TPxD
                        lut[tp + 4] = 0; // RPx — single repeat
                    }
                }
            }
        }
    }
}

fn skip_red_generic(lut: &mut [u8], layout: Layout) {
    if lut.len() < layout.body_len {
        return;
    }
    // 1. Zero LUT2 across every phase — no pixel routes there.
    for phase in 0..layout.n_phases {
        lut[layout.lut_byte(2, phase)] = 0;
    }
    // 2. Zero TP for phases where LUT0+LUT1+LUT2 are all zero — chip
    // walks every phase regardless of routing, so a dead phase still
    // costs its TP duration unless the TP bytes are zeroed.
    for phase in 0..layout.n_phases {
        if lut[layout.lut_byte(0, phase)] == 0
            && lut[layout.lut_byte(1, phase)] == 0
            && lut[layout.lut_byte(2, phase)] == 0
        {
            let tp = layout.tp_base + phase * layout.tp_stride;
            let end = (tp + layout.tp_stride).min(lut.len());
            lut[tp..end].fill(0);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────
// SSD1675A (7-phase × 5-row, 70-byte body) — patch policy
// ────────────────────────────────────────────────────────────────────────

fn patch_partial_ssd1675a(lut: &mut [u8], invert: InvertHandling) {
    patch_partial_generic(lut, SSD1675A_LAYOUT, invert);

    // Partial does not need long shake
    lut[39] = lut[39] / 4;
    lut[44] = lut[44] / 4;
    lut[49] = lut[49] * 2;
    lut[54] = lut[54] * 4;
}

fn skip_red_ssd1675a(lut: &mut [u8]) {
    skip_red_generic(lut, SSD1675A_LAYOUT);
}

// ────────────────────────────────────────────────────────────────────────
// SSD1675B (10-phase × 5-row, 99-byte body) — patch policy
// ────────────────────────────────────────────────────────────────────────

fn patch_partial_ssd1675b(lut: &mut [u8], invert: InvertHandling) {
    patch_partial_generic(lut, SSD1675B_LAYOUT, invert);
}

fn skip_red_ssd1675b(lut: &mut [u8]) {
    skip_red_generic(lut, SSD1675B_LAYOUT);
}

// ────────────────────────────────────────────────────────────────────────
// Phase 2: state buffers + per-pixel ops + bbox compute
// ────────────────────────────────────────────────────────────────────────

/// Max pixel count supported by partial-mode state buffers.  Sized
/// for the deployed 152 × 152 BornHack CyberEgg panel; raise when
/// porting to a larger variant (e.g. 296 × 176).  Drives `.bss`
/// allocation so keep it tight — every pixel costs 2 bits in each
/// of shadow/pending (4 bits total) plus 1 bit in dirty and 2 bits
/// in plane scratch (3 bits) = ~7 bits/px overhead.
pub const MAX_PANEL_PIXELS: usize = 152 * 152;

/// Bytes per packed-`Color` buffer (2 bits per pixel).
pub const PACKED_COLOR_BYTES: usize = MAX_PANEL_PIXELS / 4;

/// Bytes per dirty bitmap (1 bit per pixel).
pub const DIRTY_BYTES: usize = MAX_PANEL_PIXELS / 8;

/// Bytes per RED / BW plane scratch (1 bit per pixel each).  Built
/// fresh on every `update_partial` from `pending` + `dirty`, then
/// pushed via cmd 0x26 (RED) + cmd 0x24 (BW).
pub const PLANE_SCRATCH_BYTES: usize = MAX_PANEL_PIXELS / 8;

static SHADOW_CELL: ConstStaticCell<[u8; PACKED_COLOR_BYTES]> =
    ConstStaticCell::new([0; PACKED_COLOR_BYTES]);
static PENDING_CELL: ConstStaticCell<[u8; PACKED_COLOR_BYTES]> =
    ConstStaticCell::new([0; PACKED_COLOR_BYTES]);
static DIRTY_CELL: ConstStaticCell<[u8; DIRTY_BYTES]> = ConstStaticCell::new([0; DIRTY_BYTES]);
static RED_PLANE_CELL: ConstStaticCell<[u8; PLANE_SCRATCH_BYTES]> =
    ConstStaticCell::new([0; PLANE_SCRATCH_BYTES]);
static BW_PLANE_CELL: ConstStaticCell<[u8; PLANE_SCRATCH_BYTES]> =
    ConstStaticCell::new([0; PLANE_SCRATCH_BYTES]);

/// Rectangular region in pixel space.  Returned by
/// [`PartialState::bbox_of_dirty`] as the minimal axis-aligned
/// bounding box that contains every dirty pixel.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Rect {
    pub x: u16,
    pub y: u16,
    pub w: u16,
    pub h: u16,
}

/// Color → 2-bit packed code.  Black=0b00 / White=0b01 / Red=0b10.
/// The fourth combination (0b11) is reserved for the chip's IGNORE
/// LUT row and never stored in shadow / pending.
fn color_to_bits(c: Color) -> u8 {
    match c {
        Color::Black => 0b00,
        Color::White => 0b01,
        Color::Red => 0b10,
    }
}

/// Inverse of [`color_to_bits`].  Returns `None` for the IGNORE
/// code (0b11), which should never appear in shadow / pending.
fn color_from_bits(bits: u8) -> Option<Color> {
    match bits & 0b11 {
        0b00 => Some(Color::Black),
        0b01 => Some(Color::White),
        0b10 => Some(Color::Red),
        _ => None,
    }
}

/// Read the 2-bit color code at pixel index `idx` from a packed
/// buffer (4 pixels per byte, MSB-first).
fn packed_get(buf: &[u8], idx: usize) -> u8 {
    let byte = buf[idx >> 2];
    let shift = 6 - ((idx & 0b11) << 1);
    (byte >> shift) & 0b11
}

/// Write 2-bit color code at pixel index `idx` into a packed buffer.
fn packed_set(buf: &mut [u8], idx: usize, bits: u8) {
    let byte_idx = idx >> 2;
    let shift = 6 - ((idx & 0b11) << 1);
    let mask = 0b11u8 << shift;
    buf[byte_idx] = (buf[byte_idx] & !mask) | ((bits & 0b11) << shift);
}

/// Read 1 bit at index `idx` from a packed bitmap (MSB-first within byte).
fn bit_get(buf: &[u8], idx: usize) -> bool {
    let byte = buf[idx >> 3];
    let mask = 0x80u8 >> (idx & 0b111);
    (byte & mask) != 0
}

/// Set 1 bit at index `idx` in a packed bitmap.
fn bit_set(buf: &mut [u8], idx: usize) {
    let byte_idx = idx >> 3;
    let mask = 0x80u8 >> (idx & 0b111);
    buf[byte_idx] |= mask;
}

/// Compute the bounding box of all set bits in a dirty bitmap,
/// or `None` if no bit is set.  Pixel index = `y * cols + x`.
pub fn bbox_of_dirty(dirty: &[u8], rows: u16, cols: u16) -> Option<Rect> {
    let mut min_x = u16::MAX;
    let mut min_y = u16::MAX;
    let mut max_x = 0u16;
    let mut max_y = 0u16;
    let mut any = false;
    for y in 0..rows {
        for x in 0..cols {
            let idx = (y as usize) * (cols as usize) + (x as usize);
            if bit_get(dirty, idx) {
                if x < min_x {
                    min_x = x;
                }
                if x > max_x {
                    max_x = x;
                }
                if y < min_y {
                    min_y = y;
                }
                if y > max_y {
                    max_y = y;
                }
                any = true;
            }
        }
    }
    if any {
        Some(Rect {
            x: min_x,
            y: min_y,
            w: max_x - min_x + 1,
            h: max_y - min_y + 1,
        })
    } else {
        None
    }
}

/// MCU-side state for partial-update drive.  Holds the committed
/// panel state (`shadow`), the app's desired frame (`pending`), and
/// a per-pixel dirty bit.  Drawing and refresh run in the same task
/// (the caller holds `&mut` across the whole refresh), so `commit_refresh`
/// copies `pending` straight into `shadow` — no mid-update snapshot needed.
///
/// Buffers live in `.bss` via [`ConstStaticCell`] — zero stack
/// pressure.  `take()` is one-shot per program (single panel /
/// single instance assumed).
/// Outcome of a partial-update attempt.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UpdateKind {
    /// No dirty pixels — nothing was driven.
    NoOp,
    /// Partial refresh completed.  `bbox` is the minimal box that
    /// enclosed all dirty pixels; `had_red` reports whether any of
    /// those pixels targeted red (caller can fast-path-skip the red
    /// drive cycle on bw-only frames).
    Partial { bbox: Rect, had_red: bool },
    /// Full panel-clearing refresh ran instead (e.g., dirty
    /// percentage exceeded threshold or partial counter capped).
    Full,
    /// Refresh aborted mid-flight (HW reset issued).
    Aborted,
}

/// Runtime knobs for the partial-refresh path.  Stored on
/// `PartialState`; query / mutate via accessor methods.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PartialConfig {
    /// Force a full refresh once the cumulative count of changed pixels
    /// since the last full reaches this many full-screen equivalents
    /// (1 screen = rows × cols px).  Clears accumulated ghosting / DC
    /// bias.  Default 3.
    ///
    /// Screen switches and menu open/close mark the whole frame dirty
    /// (~1 screen each), so this is roughly "full refresh every 5
    /// screen-fulls of change".  Note: the old dirty-percentage trigger
    /// is gone — a 100 %-dirty partial (mark_all_dirty) must NOT force a
    /// full, or screen switches would never run as partials.
    pub full_after_screens: u32,
}

impl Default for PartialConfig {
    fn default() -> Self {
        Self {
            full_after_screens: 3,
        }
    }
}

pub struct PartialState {
    rows: u16,
    cols: u16,
    shadow: &'static mut [u8],
    pending: &'static mut [u8],
    dirty: &'static mut [u8],
    /// Scratch — RED plane bytes (1 bit/pixel) for the next refresh.
    /// Filled by `build_planes` from `pending` + `dirty`, then pushed
    /// via cmd 0x26.
    pub(crate) red_plane: &'static mut [u8],
    /// Scratch — BW plane bytes, pushed via cmd 0x24.
    pub(crate) bw_plane: &'static mut [u8],
    /// Set true when a refresh starts SPI traffic; cleared on
    /// successful completion.  If the future is dropped mid-update
    /// (e.g. `select!` cancellation), this stays true.  Next
    /// `update_partial` call sees the stale flag and runs a HW reset
    /// + re-init to recover the chip before proceeding.
    in_flight: bool,
    /// Count of consecutive successful partials since the last full
    /// refresh.  Compared against `config.max_partials_before_full`
    /// to decide when to force a full.
    partial_count: u32,
    /// Cumulative count of changed pixels driven since the last full
    /// refresh.  Compared against `config.full_after_screens × screen px`
    /// to decide when to force a full.  Reset on every full refresh.
    changed_px: u32,
    /// Runtime knobs — thresholds for full-refresh promotion.
    config: PartialConfig,
}

impl PartialState {
    /// Bind state to a panel of `rows × cols` pixels and take
    /// ownership of the static buffer cells.  Panics on second call.
    pub fn take(rows: u16, cols: u16) -> Self {
        Self {
            rows,
            cols,
            shadow: SHADOW_CELL.take().as_mut_slice(),
            pending: PENDING_CELL.take().as_mut_slice(),
            dirty: DIRTY_CELL.take().as_mut_slice(),
            red_plane: RED_PLANE_CELL.take().as_mut_slice(),
            bw_plane: BW_PLANE_CELL.take().as_mut_slice(),
            in_flight: false,
            partial_count: 0,
            changed_px: 0,
            config: PartialConfig::default(),
        }
    }

    pub fn config(&self) -> PartialConfig {
        self.config
    }
    pub fn set_config(&mut self, cfg: PartialConfig) {
        self.config = cfg;
    }
    pub fn in_flight(&self) -> bool {
        self.in_flight
    }
    pub fn set_in_flight(&mut self, v: bool) {
        self.in_flight = v;
    }
    pub fn partial_count(&self) -> u32 {
        self.partial_count
    }
    /// Reset the partial counter — call after a full refresh.
    pub fn reset_partial_count(&mut self) {
        self.partial_count = 0;
    }
    /// Increment the partial counter — call after each successful partial.
    pub fn bump_partial_count(&mut self) {
        self.partial_count = self.partial_count.saturating_add(1);
    }

    /// Add `n` changed pixels to the cumulative since-last-full counter.
    /// Call once per successful partial with the number of pixels driven.
    pub fn add_changed_px(&mut self, n: u32) {
        self.changed_px = self.changed_px.saturating_add(n);
    }

    /// Cumulative changed pixels since the last full refresh.
    pub fn changed_px(&self) -> u32 {
        self.changed_px
    }

    /// Reset the cumulative changed-pixel counter — call after a full refresh.
    pub fn reset_changed_px(&mut self) {
        self.changed_px = 0;
    }

    /// Decide whether the next refresh should be promoted to a full
    /// panel-clearing drive.  True once the cumulative changed-pixel count
    /// since the last full reaches `full_after_screens` full-screen
    /// equivalents.  Deliberately NOT triggered by a high single-frame
    /// dirty percentage — `mark_all_dirty` (screen switch / menu) makes a
    /// frame 100 % dirty and must still run as a partial.
    pub fn should_force_full(&self) -> bool {
        let screen_px = (self.rows as u32) * (self.cols as u32);
        let threshold = self.config.full_after_screens.saturating_mul(screen_px);
        self.changed_px >= threshold
    }

    pub fn rows(&self) -> u16 {
        self.rows
    }
    pub fn cols(&self) -> u16 {
        self.cols
    }

    fn pixel_index(&self, x: u16, y: u16) -> usize {
        (y as usize) * (self.cols as usize) + (x as usize)
    }

    /// Set pending color at `(x, y)`.  Out-of-bounds = no-op.  Marks
    /// dirty iff the new pending color differs from the shadow.
    /// Multiple writes to the same pixel collapse — dirty stays set
    /// until cleared by a successful refresh commit.
    pub fn set_pixel(&mut self, x: u16, y: u16, color: Color) {
        if x >= self.cols || y >= self.rows {
            return;
        }
        let idx = self.pixel_index(x, y);
        packed_set(self.pending, idx, color_to_bits(color));
        let shadow_bits = packed_get(self.shadow, idx);
        if shadow_bits != color_to_bits(color) {
            bit_set(self.dirty, idx);
        }
        // Note: if app writes pending back to the shadow color (after
        // first changing it), dirty stays set — pixel will get a
        // re-drive next refresh.  Cheap to over-drive a same-color
        // pixel; not worth the bookkeeping to clear dirty on revert.
    }

    /// Fill a rectangular region with `color`.  Clips to panel bounds.
    pub fn fill_rect(&mut self, x: u16, y: u16, w: u16, h: u16, color: Color) {
        for dy in 0..h {
            for dx in 0..w {
                self.set_pixel(x.saturating_add(dx), y.saturating_add(dy), color);
            }
        }
    }

    /// Read the committed shadow color at `(x, y)`.  Returns `None`
    /// if out-of-bounds or shadow holds the reserved 0b11 code.
    pub fn shadow_color(&self, x: u16, y: u16) -> Option<Color> {
        if x >= self.cols || y >= self.rows {
            return None;
        }
        color_from_bits(packed_get(self.shadow, self.pixel_index(x, y)))
    }

    /// Read the pending color at `(x, y)`.
    pub fn pending_color(&self, x: u16, y: u16) -> Option<Color> {
        if x >= self.cols || y >= self.rows {
            return None;
        }
        color_from_bits(packed_get(self.pending, self.pixel_index(x, y)))
    }

    /// True if pixel `(x, y)` is dirty (pending ≠ committed shadow).
    pub fn is_dirty(&self, x: u16, y: u16) -> bool {
        if x >= self.cols || y >= self.rows {
            return false;
        }
        bit_get(self.dirty, self.pixel_index(x, y))
    }

    /// Count dirty pixels.  O(rows × cols / 8); used by the
    /// dirty-threshold check that decides partial vs full refresh.
    pub fn dirty_count(&self) -> u32 {
        let bits = (self.rows as usize) * (self.cols as usize);
        let bytes = bits / 8;
        let mut n = 0u32;
        for &b in &self.dirty[..bytes] {
            n += b.count_ones();
        }
        n
    }

    /// Minimal bounding box of dirty pixels, or `None` if no pixel
    /// is dirty.
    pub fn bbox_of_dirty(&self) -> Option<Rect> {
        bbox_of_dirty(self.dirty, self.rows, self.cols)
    }

    /// Commit a successful refresh: for each pixel marked dirty, copy the
    /// driven `pending` colour into `shadow` and clear the dirty bit.
    ///
    /// The caller holds `&mut PartialState` across the whole refresh (drawing
    /// and refresh run in the same task; the borrow is live across every
    /// `.await`), so `pending` cannot be mutated between the start of the
    /// refresh and this commit — what was driven is exactly what is committed.
    /// (A former `sent_pending` snapshot guarded a concurrent-writer race that
    /// the exclusive borrow makes unreachable; it was removed to reclaim its
    /// full-frame buffer.)
    pub fn commit_refresh(&mut self) {
        let total = (self.rows as usize) * (self.cols as usize);
        for idx in 0..total {
            if !bit_get(self.dirty, idx) {
                continue;
            }
            packed_set(self.shadow, idx, packed_get(self.pending, idx));
            // Clear dirty bit.
            let byte_idx = idx >> 3;
            let mask = 0x80u8 >> (idx & 0b111);
            self.dirty[byte_idx] &= !mask;
        }
    }

    /// Mark every pixel dirty (e.g., post-full-refresh reset so the
    /// next partial converges any pending mutations against the
    /// freshly-driven shadow).  Cheap.
    pub fn mark_all_dirty(&mut self) {
        let bytes = (self.rows as usize) * (self.cols as usize) / 8;
        for b in self.dirty[..bytes].iter_mut() {
            *b = 0xFF;
        }
    }

    /// Black-edge halo: for every currently-dirty **black-target** pixel,
    /// also mark its 4-neighbour **white-target** pixels dirty.
    ///
    /// The strong black drive bleeds ink laterally into adjacent white
    /// pixels, fuzzing the edge.  The LUT's final phase drives white — so
    /// driving those white-edge pixels re-whitens the fringe and sharpens
    /// the black object.  In delta mode the surrounding (unchanged) white
    /// pixels wouldn't otherwise be driven; this halo adds them.
    ///
    /// Only WHITE neighbours are marked, and they drive to their own white
    /// `pending` colour — so unlike a blanket dilation this never mis-drives
    /// a held pixel to black (the bug that broke the earlier attempt).
    /// Reads a pre-dilation snapshot in `red_plane` so it never cascades
    /// past one ring; safe because `build_planes` rebuilds `red_plane`
    /// immediately after.  Call after `sync_from_planes`, before
    /// `bbox_of_dirty` / `build_planes`.
    pub fn mark_black_halo(&mut self) {
        let cols = self.cols as usize;
        let rows = self.rows as usize;
        let bits = rows * cols;
        let bytes = bits.div_ceil(8);
        // Snapshot the pre-halo dirty set so neighbour writes don't feed back.
        self.red_plane[..bytes].copy_from_slice(&self.dirty[..bytes]);
        let black = color_to_bits(Color::Black); // 0b00
        let white = color_to_bits(Color::White); // 0b01
        for idx in 0..bits {
            if !bit_get(self.red_plane, idx) {
                continue; // only originally-dirty pixels seed a halo
            }
            if packed_get(self.pending, idx) != black {
                continue; // only black-target pixels bleed
            }
            let x = idx % cols;
            let y = idx / cols;
            if x > 0 && packed_get(self.pending, idx - 1) == white {
                bit_set(self.dirty, idx - 1);
            }
            if x + 1 < cols && packed_get(self.pending, idx + 1) == white {
                bit_set(self.dirty, idx + 1);
            }
            if y > 0 && packed_get(self.pending, idx - cols) == white {
                bit_set(self.dirty, idx - cols);
            }
            if y + 1 < rows && packed_get(self.pending, idx + cols) == white {
                bit_set(self.dirty, idx + cols);
            }
        }
    }

    /// Reset both shadow + pending to `color`, clear dirty.  Use
    /// after a known full-screen clear or post-boot to align state
    /// with the actual panel.
    pub fn clear_to(&mut self, color: Color) {
        let bits = color_to_bits(color);
        let byte = (bits << 6) | (bits << 4) | (bits << 2) | bits;
        let pcb = (self.rows as usize) * (self.cols as usize) / 4;
        for b in self.shadow[..pcb].iter_mut() {
            *b = byte;
        }
        for b in self.pending[..pcb].iter_mut() {
            *b = byte;
        }
        self.clear_all_dirty();
    }

    /// Clear every dirty bit without touching shadow / pending.
    pub fn clear_all_dirty(&mut self) {
        let bytes = (self.rows as usize) * (self.cols as usize) / 8;
        for b in self.dirty[..bytes].iter_mut() {
            *b = 0;
        }
    }

    /// Direct access to the shadow buffer (packed Color, 2 bits/px).
    /// Read-only; mutate via `set_pixel` / `commit_refresh` only.
    pub fn shadow_buf(&self) -> &[u8] {
        self.shadow
    }
    /// Direct access to the pending buffer (packed Color, 2 bits/px).
    pub fn pending_buf(&self) -> &[u8] {
        self.pending
    }
    /// Direct access to the dirty bitmap (1 bit/px).
    pub fn dirty_buf(&self) -> &[u8] {
        self.dirty
    }
    /// Direct access to the RED-plane scratch buffer.
    pub fn red_plane_buf(&self) -> &[u8] {
        self.red_plane
    }
    /// Direct access to the BW-plane scratch buffer.
    pub fn bw_plane_buf(&self) -> &[u8] {
        self.bw_plane
    }

    /// Populate `red_plane` + `bw_plane` from `pending` + `dirty`.
    /// Default per-pixel = IGNORE (RED=1, BW=1) → routes through
    /// LUT3 which the patched LUT has zeroed (no drive).  Dirty
    /// pixels override with the encoding from the spec table:
    ///
    /// | Color | RED bit | BW bit | LUT row | Action       |
    /// |-------|---------|--------|---------|--------------|
    /// | Black |   0     |   0    | LUT0    | drive black  |
    /// | White |   0     |   1    | LUT1    | drive white  |
    /// | Red   |   1     |   0    | LUT2    | drive red    |
    ///
    /// Returns `had_red` — true if any dirty pixel targets red.
    /// Caller may use this to fast-path-skip red-drive cycles.
    pub fn build_planes(&mut self) -> bool {
        build_planes(
            self.pending,
            self.dirty,
            self.rows,
            self.cols,
            self.red_plane,
            self.bw_plane,
        )
    }

    /// Build planes from `pending` for a FULL refresh using the
    /// `graphics.rs` color convention (the encoding the OTP LUT
    /// expects):
    ///
    /// | Color | BW bit | RED bit | OTP LUT row routing |
    /// |-------|--------|---------|----------------------|
    /// | Black |   0    |   0     | LUT0                 |
    /// | White |   1    |   0     | LUT1                 |
    /// | Red   |   1    |   1     | LUT3                 |
    ///
    /// Every pixel is encoded, regardless of dirty.  The OTP LUT
    /// drives every row with non-zero waveform → whole panel
    /// refreshes.  Used by `Display::update_full_from_state`.
    pub fn build_planes_full(&mut self) {
        build_planes_full(
            self.pending,
            self.rows,
            self.cols,
            self.red_plane,
            self.bw_plane,
        );
    }
}

/// Bridge: copy a frame from `GraphicDisplay`-style planes
/// (separate `black` + `red` 1-bit-per-pixel bitmaps using the
/// `graphics.rs` convention) into the `PartialState`'s pending
/// buffer.  Marks dirty for every pixel where the new color
/// differs from shadow.
///
/// Use this when the app draws via embedded-graphics into the
/// existing 1-bit bitmaps and the partial-update driver needs
/// the packed-Color representation.
///
/// `black` / `red` are MSB-first, `rows * cols / 8` bytes each.
/// Pixel convention matches `graphics.rs::set_pixel`:
///   * White: black bit=1, red bit=0
///   * Black: black bit=0, red bit=0
///   * Red:   black bit=1, red bit=1
pub fn sync_from_planes(state: &mut PartialState, black: &[u8], red: &[u8]) {
    let rows = state.rows();
    let cols = state.cols();
    for y in 0..rows {
        for x in 0..cols {
            let idx = (y as usize) * (cols as usize) + (x as usize);
            let byte_idx = idx >> 3;
            let mask = 0x80u8 >> (idx & 0b111);
            let bw_bit = black.get(byte_idx).copied().unwrap_or(0) & mask != 0;
            let r_bit = red.get(byte_idx).copied().unwrap_or(0) & mask != 0;
            let color = match (bw_bit, r_bit) {
                (true, true) => Color::Red,
                (true, false) => Color::White,
                // (false, true) is "red but BW=0" — never produced
                // by graphics.rs::set_pixel; treat as black.
                (false, _) => Color::Black,
            };
            state.set_pixel(x, y, color);
        }
    }
}

/// Free-function plane builder for FULL refresh using the
/// `graphics.rs` Color convention.  Every pixel encoded (no IGNORE).
pub fn build_planes_full(
    pending: &[u8],
    rows: u16,
    cols: u16,
    red_out: &mut [u8],
    bw_out: &mut [u8],
) {
    let bytes = (rows as usize) * (cols as usize) / 8;
    for b in red_out[..bytes].iter_mut() {
        *b = 0;
    }
    for b in bw_out[..bytes].iter_mut() {
        *b = 0;
    }

    let pixels = (rows as usize) * (cols as usize);
    for idx in 0..pixels {
        let color_bits = packed_get(pending, idx);
        // graphics.rs convention:
        //   Black: bw=0, red=0
        //   White: bw=1, red=0
        //   Red:   bw=1, red=1
        let (red, bw) = match color_bits {
            0b00 => (false, false), // Black
            0b01 => (false, true),  // White
            0b10 => (true, true),   // Red — both bits set
            _ => (false, false),
        };
        let byte_idx = idx >> 3;
        let mask = 0x80u8 >> (idx & 0b111);
        if red {
            red_out[byte_idx] |= mask;
        }
        if bw {
            bw_out[byte_idx] |= mask;
        }
    }
}

/// Free-function plane builder.  Exposed for unit testing without
/// constructing a full `PartialState`.  Returns `had_red`.
pub fn build_planes(
    pending: &[u8],
    dirty: &[u8],
    rows: u16,
    cols: u16,
    red_out: &mut [u8],
    bw_out: &mut [u8],
) -> bool {
    // Default = IGNORE class (RED=1, BW=1) on every pixel → all-set
    // bytes.  Bounds: rows × cols / 8 bytes of plane data.
    let bytes = (rows as usize) * (cols as usize) / 8;
    for b in red_out[..bytes].iter_mut() {
        *b = 0xFF;
    }
    for b in bw_out[..bytes].iter_mut() {
        *b = 0xFF;
    }

    let pixels = (rows as usize) * (cols as usize);
    let mut had_red = false;
    for idx in 0..pixels {
        if !bit_get(dirty, idx) {
            continue;
        }
        let color_bits = packed_get(pending, idx);
        // (red_bit_value, bw_bit_value) — what to PUT in the planes.
        let (red, bw) = match color_bits {
            0b00 => (false, false), // Black → LUT0
            0b01 => (false, true),  // White → LUT1
            0b10 => {
                had_red = true;
                (true, false)
            } // Red → LUT2
            _ => continue,          // 0b11 unreachable in pending
        };
        let byte_idx = idx >> 3;
        let mask = 0x80u8 >> (idx & 0b111);
        // Default both planes are 0xFF (= bit set).  Clear bit
        // where the target color wants a 0.
        if !red {
            red_out[byte_idx] &= !mask;
        }
        if !bw {
            bw_out[byte_idx] &= !mask;
        }
    }
    had_red
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SSD1675A-specific compatibility shims for the tests written
    /// against the original (variant-free) API.
    const N_PHASES: usize = SSD1675A_LAYOUT.n_phases;
    const TP_BASE: usize = SSD1675A_LAYOUT.tp_base;
    const TP_STRIDE: usize = SSD1675A_LAYOUT.tp_stride;
    const LUT3_OFFSETS: [usize; N_PHASES] = [21, 22, 23, 24, 25, 26, 27];
    const fn lut_byte(row: usize, phase: usize) -> usize {
        row * SSD1675A_LAYOUT.phases_per_row + phase
    }

    #[test]
    fn color_bits_match_spec_table() {
        assert_eq!(color_to_ram_bits(Color::Black), (false, false));
        assert_eq!(color_to_ram_bits(Color::White), (false, true));
        assert_eq!(color_to_ram_bits(Color::Red), (true, false));
    }

    #[test]
    fn lut3_offsets_zeroed() {
        let mut lut = [0xAAu8; 70];
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675, InvertHandling::Refine);
        for &off in &LUT3_OFFSETS {
            assert_eq!(lut[off], 0, "LUT3 byte at offset {} not zeroed", off);
        }
    }

    #[test]
    fn shake_phase_preserved_for_dirty_pixels() {
        // Shake phase 1: LUT0/LUT1/LUT2 match nonzero.  After patch:
        // LUT0+LUT1 STAY (B↔W dirty pixels get OTP shake), LUT2
        // zeroed (red dirty pixels skip shake).  TP1 halved + RP1
        // zeroed so shake runs once at reduced amplitude.
        let mut lut = [0u8; 70];
        lut[1] = 0x55; // LUT0 phase 1
        lut[8] = 0x55; // LUT1 phase 1
        lut[15] = 0x55; // LUT2 phase 1
        let tp1 = 35 + 1 * 5;
        for off in tp1..tp1 + 5 {
            lut[off] = 0xCC;
        }
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675, InvertHandling::Refine);
        assert_eq!(lut[1], 0x55, "LUT0 shake preserved");
        assert_eq!(lut[8], 0x55, "LUT1 shake preserved");
        assert_eq!(lut[15], 0, "LUT2 shake zeroed (red skips shake)");
        assert_eq!(lut[tp1], 0x66, "TPxA halved");
        assert_eq!(lut[tp1 + 1], 0x66, "TPxB halved");
        assert_eq!(lut[tp1 + 2], 0x66, "TPxC halved");
        assert_eq!(lut[tp1 + 3], 0x66, "TPxD halved");
        assert_eq!(lut[tp1 + 4], 0, "RPx zeroed");
    }

    #[test]
    fn drive_phase_not_zeroed() {
        // Phase 3: LUT0/LUT1/LUT2 differ → not a shake.  Bytes stay.
        let mut lut = [0u8; 70];
        lut[3] = 0xC0; // LUT0 phase 3
        lut[10] = 0x40; // LUT1 phase 3
        lut[17] = 0x80; // LUT2 phase 3
        // LUT3 phase 3 = byte 24 — will be zeroed by step 1.
        lut[24] = 0xFF;
        // TP3 = bytes 50..55.  Should stay if not a shake.
        let tp3 = 35 + 3 * 5;
        for off in tp3..tp3 + 5 {
            lut[off] = 0x11;
        }
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675, InvertHandling::Refine);
        assert_eq!(lut[3], 0xC0);
        assert_eq!(lut[10], 0x40);
        assert_eq!(lut[17], 0x80);
        assert_eq!(lut[24], 0, "LUT3 phase 3 not zeroed");
        for off in tp3..tp3 + 5 {
            assert_eq!(lut[off], 0x11, "TP3 byte at {} overwritten", off);
        }
    }

    #[test]
    fn lut4_vcom_zeroed() {
        // LUT4 (VCOM) bytes 28..35 must be zeroed — OTP VCOM
        // modulation otherwise applies to IGNORE-class pixels and
        // accumulates per-refresh DC bias.
        let mut lut = [0u8; 70];
        for off in 28..35 {
            lut[off] = 0x77;
        }
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675, InvertHandling::Refine);
        for off in 28..35 {
            assert_eq!(lut[off], 0, "VCOM byte at {} not zeroed", off);
        }
    }

    #[test]
    fn skip_red_zeros_lut2_and_red_only_tp() {
        // Phase 2 = red-only drive (LUT2 nonzero, LUT0/1 zero).
        // Phase 4 = mixed BW (LUT0/1 nonzero + LUT2 nonzero).
        // After skip_red: phase 2 fully dead (TP zeroed), phase 4
        // keeps TP (LUT0/1 still drive) but LUT2 byte zeroed.
        let mut lut = [0u8; 70];
        lut[lut_byte(2, 2)] = 0xC0;
        lut[lut_byte(0, 4)] = 0x40;
        lut[lut_byte(1, 4)] = 0x80;
        lut[lut_byte(2, 4)] = 0x10;
        let tp2 = TP_BASE + 2 * TP_STRIDE;
        let tp4 = TP_BASE + 4 * TP_STRIDE;
        for off in tp2..tp2 + TP_STRIDE {
            lut[off] = 0x99;
        }
        for off in tp4..tp4 + TP_STRIDE {
            lut[off] = 0xAA;
        }
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675, InvertHandling::Refine);
        patch_lut_skip_red(&mut lut, DisplayVariant::Ssd1675);
        assert_eq!(lut[lut_byte(2, 2)], 0, "LUT2 phase 2 zeroed");
        assert_eq!(lut[lut_byte(2, 4)], 0, "LUT2 phase 4 zeroed");
        for off in tp2..tp2 + TP_STRIDE {
            assert_eq!(lut[off], 0, "TP2 dead-phase byte {} not zeroed", off);
        }
        for off in tp4..tp4 + TP_STRIDE {
            assert_eq!(
                lut[off], 0xAA,
                "TP4 mixed-phase byte {} unexpectedly modified",
                off
            );
        }
        assert_eq!(lut[lut_byte(0, 4)], 0x40, "LUT0 phase 4 preserved");
        assert_eq!(lut[lut_byte(1, 4)], 0x80, "LUT1 phase 4 preserved");
    }

    #[test]
    fn ssd1675b_layout_lut3_and_lut4_zeroed() {
        // SSD1675B: 100-byte effective body (driver pushes 99).  LUT3
        // = bytes 30..40, LUT4 (VCOM) = bytes 40..50.  Patch must zero
        // both per the same partial-mode rules as SSD1675A.
        let mut lut = [0xAAu8; 100];
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675B, InvertHandling::Refine);
        for off in 30..40 {
            assert_eq!(lut[off], 0, "1675B LUT3 byte {} not zeroed", off);
        }
        for off in 40..50 {
            assert_eq!(lut[off], 0, "1675B LUT4 byte {} not zeroed", off);
        }
    }

    #[test]
    fn ssd1675b_shake_phase_refined_with_tp() {
        // SSD1675B shake phase 1 lives at byte 1 (LUT0), 11 (LUT1), 21
        // (LUT2).  TP1 = bytes 55..60.  Same refinement as A: LUT0/1
        // preserved, LUT2 zeroed, TP halved + RP zeroed.
        let mut lut = [0u8; 100];
        lut[1] = 0x40; // LUT0 phase 1
        lut[11] = 0x40; // LUT1 phase 1
        lut[21] = 0x40; // LUT2 phase 1
        let tp1 = 50 + 1 * 5;
        for off in tp1..tp1 + 5 {
            lut[off] = 0x80;
        }
        patch_lut_for_partial(&mut lut, DisplayVariant::Ssd1675B, InvertHandling::Refine);
        assert_eq!(lut[1], 0x40, "1675B LUT0 shake preserved");
        assert_eq!(lut[11], 0x40, "1675B LUT1 shake preserved");
        assert_eq!(lut[21], 0, "1675B LUT2 shake zeroed");
        assert_eq!(lut[tp1], 0x40, "1675B TPxA halved");
        assert_eq!(lut[tp1 + 4], 0, "1675B RPx zeroed");
    }

    // ── Phase 2 — packing helpers ───────────────────────────────────

    #[test]
    fn color_bits_roundtrip() {
        for &c in &[Color::Black, Color::White, Color::Red] {
            let bits = color_to_bits(c);
            assert_eq!(color_from_bits(bits), Some(c));
        }
        // Reserved 0b11 code maps to None.
        assert_eq!(color_from_bits(0b11), None);
    }

    #[test]
    fn packed_get_set_per_position() {
        let mut buf = [0u8; 4];
        // Set pixel 0 (top of byte 0) to 0b10
        packed_set(&mut buf, 0, 0b10);
        assert_eq!(buf[0], 0b10_00_00_00);
        assert_eq!(packed_get(&buf, 0), 0b10);

        // Set pixel 1 (next slot) to 0b01
        packed_set(&mut buf, 1, 0b01);
        assert_eq!(buf[0], 0b10_01_00_00);
        assert_eq!(packed_get(&buf, 1), 0b01);

        // Set pixel 2 to 0b11 then back to 0b00 — neighbours untouched
        packed_set(&mut buf, 2, 0b11);
        packed_set(&mut buf, 3, 0b10);
        assert_eq!(buf[0], 0b10_01_11_10);
        packed_set(&mut buf, 2, 0b00);
        assert_eq!(buf[0], 0b10_01_00_10);

        // Byte boundary: pixel 4 lives in byte 1
        packed_set(&mut buf, 4, 0b11);
        assert_eq!(buf[1], 0b11_00_00_00);
        assert_eq!(packed_get(&buf, 4), 0b11);

        // Far end: pixel 15 = byte 3 sub-3
        packed_set(&mut buf, 15, 0b10);
        assert_eq!(buf[3], 0b00_00_00_10);
    }

    #[test]
    fn bit_get_set_msb_first() {
        let mut buf = [0u8; 2];
        bit_set(&mut buf, 0);
        assert_eq!(buf[0], 0b1000_0000);
        bit_set(&mut buf, 7);
        assert_eq!(buf[0], 0b1000_0001);
        bit_set(&mut buf, 8);
        assert_eq!(buf[1], 0b1000_0000);

        assert!(bit_get(&buf, 0));
        assert!(bit_get(&buf, 7));
        assert!(bit_get(&buf, 8));
        assert!(!bit_get(&buf, 1));
        assert!(!bit_get(&buf, 9));
    }

    // ── Phase 2 — bbox_of_dirty ─────────────────────────────────────

    fn make_dirty(rows: u16, cols: u16) -> alloc::vec::Vec<u8> {
        let bytes = ((rows as usize) * (cols as usize) + 7) / 8;
        alloc::vec![0u8; bytes]
    }

    fn set_xy(dirty: &mut [u8], cols: u16, x: u16, y: u16) {
        let idx = (y as usize) * (cols as usize) + (x as usize);
        bit_set(dirty, idx);
    }

    #[test]
    fn bbox_none_when_empty() {
        let dirty = make_dirty(16, 16);
        assert_eq!(bbox_of_dirty(&dirty, 16, 16), None);
    }

    #[test]
    fn bbox_single_pixel() {
        let mut dirty = make_dirty(16, 16);
        set_xy(&mut dirty, 16, 5, 7);
        assert_eq!(
            bbox_of_dirty(&dirty, 16, 16),
            Some(Rect {
                x: 5,
                y: 7,
                w: 1,
                h: 1
            })
        );
    }

    #[test]
    fn bbox_spans_extremes() {
        let mut dirty = make_dirty(32, 32);
        set_xy(&mut dirty, 32, 0, 0);
        set_xy(&mut dirty, 32, 31, 31);
        set_xy(&mut dirty, 32, 10, 20);
        assert_eq!(
            bbox_of_dirty(&dirty, 32, 32),
            Some(Rect {
                x: 0,
                y: 0,
                w: 32,
                h: 32
            })
        );
    }

    #[test]
    fn bbox_localised_cluster() {
        let mut dirty = make_dirty(64, 64);
        for y in 10..15 {
            for x in 20..30 {
                set_xy(&mut dirty, 64, x, y);
            }
        }
        assert_eq!(
            bbox_of_dirty(&dirty, 64, 64),
            Some(Rect {
                x: 20,
                y: 10,
                w: 10,
                h: 5
            })
        );
    }

    // ── Phase 2 — PartialState integration ──────────────────────────
    //
    // PartialState::take() consumes ConstStaticCells (one-shot per
    // program).  Tests below allocate buffers via Vec + Box::leak so
    // each test gets a fresh state without colliding on the singleton.

    fn fresh_state(rows: u16, cols: u16) -> PartialState {
        let pixels = (rows as usize) * (cols as usize);
        let leak = |v: alloc::vec::Vec<u8>| alloc::boxed::Box::leak(v.into_boxed_slice());
        let shadow = leak(alloc::vec![0x55u8; pixels / 4]); // all-white packed
        let pending = leak(alloc::vec![0x55u8; pixels / 4]);
        let dirty = leak(alloc::vec![0u8; pixels / 8]);
        let red_plane = leak(alloc::vec![0u8; pixels / 8]);
        let bw_plane = leak(alloc::vec![0u8; pixels / 8]);
        PartialState {
            rows,
            cols,
            shadow,
            pending,
            dirty,
            red_plane,
            bw_plane,
            in_flight: false,
            partial_count: 0,
            changed_px: 0,
            config: PartialConfig::default(),
        }
    }

    #[test]
    fn set_pixel_marks_dirty_when_differs() {
        let mut s = fresh_state(64, 64);
        assert_eq!(s.shadow_color(10, 10), Some(Color::White));
        s.set_pixel(10, 10, Color::Black);
        assert!(s.is_dirty(10, 10));
        assert_eq!(s.pending_color(10, 10), Some(Color::Black));
        assert_eq!(s.shadow_color(10, 10), Some(Color::White));
    }

    #[test]
    fn set_pixel_same_as_shadow_does_not_dirty() {
        let mut s = fresh_state(64, 64);
        s.set_pixel(5, 5, Color::White);
        assert!(!s.is_dirty(5, 5));
    }

    #[test]
    fn out_of_bounds_set_pixel_is_noop() {
        let mut s = fresh_state(32, 32);
        s.set_pixel(32, 0, Color::Black);
        s.set_pixel(0, 32, Color::Black);
        assert_eq!(s.dirty_count(), 0);
    }

    #[test]
    fn fill_rect_marks_region_dirty() {
        let mut s = fresh_state(64, 64);
        s.fill_rect(0, 0, 8, 8, Color::Black);
        for y in 0..8 {
            for x in 0..8 {
                assert!(s.is_dirty(x, y));
                assert_eq!(s.pending_color(x, y), Some(Color::Black));
            }
        }
        assert_eq!(s.dirty_count(), 64);
    }

    #[test]
    fn bbox_method_returns_dirty_extent() {
        let mut s = fresh_state(64, 64);
        s.fill_rect(20, 30, 4, 2, Color::Red);
        assert_eq!(
            s.bbox_of_dirty(),
            Some(Rect {
                x: 20,
                y: 30,
                w: 4,
                h: 2
            })
        );
    }

    #[test]
    fn commit_copies_pending_to_shadow_and_clears_dirty() {
        let mut s = fresh_state(64, 64);
        s.set_pixel(40, 40, Color::Black);
        assert!(s.is_dirty(40, 40));
        s.commit_refresh();
        assert!(!s.is_dirty(40, 40));
        assert_eq!(s.shadow_color(40, 40), Some(Color::Black));
    }

    #[test]
    fn mark_all_dirty_sets_every_bit() {
        let mut s = fresh_state(32, 32);
        s.mark_all_dirty();
        assert_eq!(s.dirty_count() as usize, 32 * 32);
    }

    #[test]
    fn clear_all_dirty_clears() {
        let mut s = fresh_state(32, 32);
        s.fill_rect(0, 0, 5, 5, Color::Black);
        assert!(s.dirty_count() > 0);
        s.clear_all_dirty();
        assert_eq!(s.dirty_count(), 0);
    }

    // ── Phase 3 — build_planes ──────────────────────────────────────

    fn pixel_bits(red: &[u8], bw: &[u8], cols: u16, x: u16, y: u16) -> (bool, bool) {
        let idx = (y as usize) * (cols as usize) + (x as usize);
        let byte = idx >> 3;
        let mask = 0x80u8 >> (idx & 0b111);
        ((red[byte] & mask) != 0, (bw[byte] & mask) != 0)
    }

    #[test]
    fn build_planes_defaults_to_ignore() {
        let mut s = fresh_state(32, 32);
        // No dirty pixels.
        let had_red = s.build_planes();
        assert!(!had_red);
        // Every plane byte should be 0xFF (RED=1, BW=1 → IGNORE).
        let bytes = 32 * 32 / 8;
        for b in &s.red_plane[..bytes] {
            assert_eq!(*b, 0xFF);
        }
        for b in &s.bw_plane[..bytes] {
            assert_eq!(*b, 0xFF);
        }
    }

    #[test]
    fn build_planes_encodes_black_white_red_per_spec() {
        let mut s = fresh_state(32, 32);
        // Start from a Black baseline so all three target colors
        // (Black, White, Red) generate a non-trivial transition.
        s.clear_to(Color::Black);
        // Pixel (1, 0) → Black: same as shadow, won't dirty (covered by
        // the unchanged-pixels test below).  Use pixel (1, 0) for Red
        // from a White local shadow set via set_pixel...
        // Simpler: clear to White, then set Black + Red on two pixels;
        // for White, clear THAT pixel to Black first via set_pixel
        // (which dirties), then set to White (still dirty because
        // dirty is sticky).
        s.clear_to(Color::White);
        s.set_pixel(1, 0, Color::Black); // dirty: Black target
        s.set_pixel(2, 0, Color::Black); // first transition: dirty
        s.set_pixel(2, 0, Color::White); // sticky dirty: White target
        s.set_pixel(3, 0, Color::Red); // dirty: Red target
        let had_red = s.build_planes();
        assert!(had_red);
        // Per spec encoding:
        // Black → (RED=0, BW=0)
        assert_eq!(
            pixel_bits(s.red_plane, s.bw_plane, 32, 1, 0),
            (false, false)
        );
        // White → (RED=0, BW=1)
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 2, 0), (false, true));
        // Red → (RED=1, BW=0)
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 3, 0), (true, false));
        // Untouched pixel (0, 0) → IGNORE = (1, 1)
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 0, 0), (true, true));
    }

    #[test]
    fn build_planes_unchanged_pixels_stay_ignore() {
        let mut s = fresh_state(32, 32);
        // Set a white pixel — pending matches shadow (initial = White),
        // so dirty stays 0.
        s.set_pixel(5, 5, Color::White);
        assert!(!s.is_dirty(5, 5));
        s.build_planes();
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 5, 5), (true, true));
    }

    #[test]
    fn mark_black_halo_whitens_white_neighbours_of_black() {
        let mut s = fresh_state(32, 32);
        s.clear_to(Color::White); // white baseline, committed (not dirty)
        s.set_pixel(5, 5, Color::Black); // single black change
        assert!(s.is_dirty(5, 5));
        assert!(!s.is_dirty(4, 5));
        s.mark_black_halo();
        // White 4-neighbours of the black pixel are now dirty.
        assert!(s.is_dirty(4, 5));
        assert!(s.is_dirty(6, 5));
        assert!(s.is_dirty(5, 4));
        assert!(s.is_dirty(5, 6));
        // Centre stays dirty; diagonals untouched (4-neighbour only).
        assert!(s.is_dirty(5, 5));
        assert!(!s.is_dirty(4, 4));
    }

    #[test]
    fn mark_black_halo_only_seeds_from_black() {
        // A dirty WHITE pixel must not seed a halo.
        let mut s = fresh_state(32, 32);
        s.clear_to(Color::White);
        s.set_pixel(10, 10, Color::Black);
        s.set_pixel(10, 10, Color::White); // sticky-dirty but pending = white
        s.mark_black_halo();
        assert!(!s.is_dirty(9, 10));
        assert!(!s.is_dirty(11, 10));
    }

    #[test]
    fn build_planes_had_red_false_when_no_red_dirty() {
        let mut s = fresh_state(32, 32);
        s.fill_rect(0, 0, 8, 8, Color::Black);
        s.fill_rect(8, 0, 8, 8, Color::White);
        let had_red = s.build_planes();
        assert!(!had_red);
    }

    // ── Phase 4 — in-flight + counter ────────────────────────────────

    #[test]
    fn in_flight_default_false() {
        let s = fresh_state(32, 32);
        assert!(!s.in_flight());
    }

    #[test]
    fn in_flight_setter_roundtrip() {
        let mut s = fresh_state(32, 32);
        s.set_in_flight(true);
        assert!(s.in_flight());
        s.set_in_flight(false);
        assert!(!s.in_flight());
    }

    #[test]
    fn partial_counter_bump_and_reset() {
        let mut s = fresh_state(32, 32);
        assert_eq!(s.partial_count(), 0);
        s.bump_partial_count();
        s.bump_partial_count();
        assert_eq!(s.partial_count(), 2);
        s.reset_partial_count();
        assert_eq!(s.partial_count(), 0);
    }

    // ── Phase 5 — threshold + full-build encoding ────────────────────

    #[test]
    fn should_force_full_after_n_screens() {
        let mut s = fresh_state(32, 32); // 1024 px / screen
        s.set_config(PartialConfig {
            full_after_screens: 3,
        });
        // Under 3 screens of cumulative change → no force.
        s.add_changed_px(1024 * 2);
        assert!(!s.should_force_full());
        s.add_changed_px(1023);
        assert!(!s.should_force_full());
        // Cross the 3-screen (3072 px) threshold.
        s.add_changed_px(1);
        assert!(s.should_force_full());
        // Reset (as a full refresh would) clears it.
        s.reset_changed_px();
        assert!(!s.should_force_full());
    }

    #[test]
    fn full_dirty_partial_does_not_force_full() {
        // mark_all_dirty makes the frame 100 % dirty but that alone must
        // NOT force a full — screen switches run as partials.
        let mut s = fresh_state(32, 32);
        s.set_config(PartialConfig {
            full_after_screens: 5,
        });
        s.mark_all_dirty();
        assert_eq!(s.dirty_count(), 1024);
        assert!(!s.should_force_full());
    }

    #[test]
    fn build_planes_full_graphics_convention() {
        let mut s = fresh_state(32, 32);
        // Pending is initially 0x55 = all-White (0b01).
        s.set_pixel(0, 0, Color::Black);
        s.set_pixel(1, 0, Color::Red);
        // (2, 0) stays White via initial pending; will encode as graphics
        // White (bw=1, red=0).
        s.build_planes_full();
        // Black: bw=0, red=0
        assert_eq!(
            pixel_bits(s.red_plane, s.bw_plane, 32, 0, 0),
            (false, false)
        );
        // Red: bw=1, red=1
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 1, 0), (true, true));
        // White: bw=1, red=0
        assert_eq!(pixel_bits(s.red_plane, s.bw_plane, 32, 2, 0), (false, true));
    }
}

#[cfg(test)]
extern crate alloc;

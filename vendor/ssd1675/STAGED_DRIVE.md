# Staged neighbourhood-aware drive

The `staged` module (feature `staged`) splits one logical SSD1675 B/W update into
several short, equal-length drive stages so that edge and isolated pixels receive
extra cumulative impulse without re-driving stable interior pixels. The crate
provides only the **mechanics** — LUT encoding (`StageLut`), per-stage plane
packing (`pack_stage`), neighbourhood classification
(`distance_transform`/`classify`/`Class`), ledger math
(`ledger_apply_stage`/`ledger_bias_transition`), and a thin stage executor
(`upload_lut`/`upload_planes`/`trigger_stage`/`Region`). All **policy and state**
(the ledger buffer, the per-update schedule, and the run-to-completion loop) lives
in the firmware (`src/fw/epd_staged.rs`); see the [Stage schedule](#stage-schedule)
section.

## RAM-bit → group mapping

Each pixel selects one of five waveform LUT rows through its two RAM-plane bits.
The controller indexes the LUT row by `group = RED*2 + BW`. `StageAction::ram_bits`
(`src/staged/mod.rs`) returns `(red_bit, bw_bit)` for a stage action, exactly
mirroring `partial::color_to_ram_bits` (`src/partial.rs`):

| `StageAction` | RED | BW | `group` | LUT row | Action            |
|---------------|-----|----|---------|---------|-------------------|
| `DriveBlack`  |  0  |  0 |    0    | LUT0    | drive to black    |
| `DriveWhite`  |  0  |  1 |    1    | LUT1    | drive to white    |
| `DriveRed`    |  1  |  0 |    2    | LUT2    | drive to red      |
| `NoOp`        |  1  |  1 |    3    | LUT3    | ignore (no drive) |
|               |     |    |    4    | LUT4    | VCOM              |

`pack_stage` writes these two bits into the BW plane (cmd `0x24`) and the RED plane
(cmd `0x26`), MSB-first and row-major, matching the existing partial-update buffer
convention.

### Red routing differs between the two paths

The custom-LUT (staged / partial) path and the OTP full-update path route red to
**different** LUT rows:

- **Custom-LUT / partial path** — red is `(RED=1, BW=0)` → **LUT2**
  (`partial::color_to_ram_bits`, `StageAction::DriveRed`). The stage's own LUT2 row
  carries the red waveform.
- **OTP full-update path** — `graphics.rs::set_pixel` encodes red as both buffer
  bits set, i.e. `(BW=1, RED=1)` → **LUT3** (see `partial::build_planes_full`,
  `0b10 => (true, true)`). The factory OTP LUT drives every row including LUT3, so
  red is driven from the OTP's LUT3 row on a full refresh.

The staged path therefore must place its red waveform in LUT2, not LUT3, and keeps
LUT3 reserved as the genuine no-op row.

## Per-variant LUT layout

`StageLut::encode` (`src/staged/lut.rs`) emits the LUT **body** sized for the
target controller. Geometry comes from the `Layout` constants in `src/partial.rs`:

| Variant   | `n_phases` | `body_len` | `tp_base` | `tp_stride` |
|-----------|-----------|-----------|-----------|-------------|
| SSD1675A  |     7     |    70     |    35     |      5      |
| SSD1675B  |    10     |    99     |    50     |      5      |

The waveform region is 5 LUT rows × `n_phases` selector bytes, laid out row-major.
The byte offset for a `(row, phase)` selector is
`Layout::lut_byte(row, phase) = row * phases_per_row + phase` (`partial.rs`).
The five rows, in order, are:

| Row | LUT  | Source field          |
|-----|------|-----------------------|
|  0  | LUT0 | `StageLut.black`      |
|  1  | LUT1 | `StageLut.white`      |
|  2  | LUT2 | `StageLut.red`        |
|  3  | LUT3 | NoOp — forced to zero |
|  4  | LUT4 | `StageLut.vcom`       |

The TP timing region follows the waveform region at `tp_base`, one
`(A, B, C, D, RP)` entry per phase at stride `tp_stride`.

Invariants enforced by `encode`:

- The whole body is zeroed first, so phases beyond `n_phases` (and the unused
  trailing byte of the SSD1675B TP region) stay zero.
- **LUT3 (NoOp) is always zero** — `encode` writes a static `ZERO_GROUP` for row 3
  regardless of inputs, so NoOp pixels receive no drive.
- **The red row is zeroed when `StageLut.red == None`** — `encode` substitutes
  `ZERO_GROUP` for row 2, giving a stage with no red drive (the common B/W case).

`MAX_PHASES` (10) and `MAX_BODY` (99) bound the per-variant maxima; the
`StageLut.tp` and `GroupWaveform.phases` arrays are sized to `MAX_PHASES`.

## Temperature-table format

The stage body carries **per-phase selector bytes only** — it says *which* source
rail each phase uses, not the rail magnitude. The actual drive **voltages** are
supplied by the temperature trailer, owned entirely by the driver.

`Display::apply_lut_trailer` (`src/display.rs`) reads `active_temp_c10` (set via
`set_active_temperature`), maps it to a band index with `band_idx()` (each band
spans a 4 °C window), and pushes the per-band `VoltageProfile` via the gate-voltage
(`0x03`), source-voltage (`0x04`), VCOM (`0x2C`), dummy-line (`0x3A`), and
gate-line (`0x3B`) commands:

- **SSD1675A** — voltages come from A's own probed OTP trailer bytes 70–75
  (VGH, VSH1, VSH2, VSL, dummy, gate); VCOM is the constant `0x50` (A's OTP has no
  VCOM byte).
- **SSD1675B** — voltages come from B's probed OTP trailer bytes 100–106
  (VGH, VSH1, VSH2, VSL, VCOM, dummy, gate).

The lookup is a cheap integer index into the registered table, so there is **no
per-update voltage re-download** — the table is registered once and indexed by the
current temperature on each refresh. The staged executor stays state-free: it
injects the trailer through the `apply_trailer` async closure passed to
`upload_lut`, which the firmware wires to the driver's `apply_lut_trailer`. So the
body (selectors) and the trailer (magnitudes) are uploaded together by
`upload_lut`, but only the body is stage-specific.

## Ledger semantics

The ledger is one `i8` per pixel holding a **commanded feed-forward impulse** — the
net signed drive *requested* across stages, not a measured coulomb count. The sign
convention is the firmware's; the crate keeps it consistent across apply and bias
(`src/staged/ledger.rs`):

- **Apply per stage** — `ledger_apply_stage(ledger, i, impulse)` saturating-adds the
  stage's signed impulse to pixel `i` for every pixel that stage drives.
- **Discharge only on a real transition** — `ledger_bias_transition(ledger, i)`
  reads the stored remainder as the bias to fold into the next drive, then zeros the
  entry. It is called **only** when a pixel actually transitions colour; a static
  pixel's accumulated impulse is never spent, so no balancing pulse is ever emitted
  at rest.
- **Zeroed at boot** — the firmware clears the ledger buffer at startup so the panel
  begins from a known-balanced state.

## Stage schedule

The schedule constants and the run-to-completion loop live **firmware-side** in
`src/fw/epd_staged.rs`, not in this crate — the crate only provides the `Class`
enum the schedule keys on. The default policy:

- **3 equal stages of 100 ms** (`STAGE_LEN_MS = 100`, `STAGE_COUNT = 3`), with a
  hard cap of **4** stages (`MAX_STAGES`).
- **Stage membership** (`class_in_stage`):
  - S0 — all dirty pixels.
  - S1 — `Edge` and `Isolated` only.
  - S2 — `Isolated` only.
- **Cumulative impulse** (number of stages a class is driven in,
  `class_impulse`): `Interior` ×1, `Edge` ×2, `Isolated` ×3.
- **Imbalance ceiling** — the extra drive any class can accrue over `Interior` is
  bounded by `(Isolated − Interior) × STAGE_LEN_MS = 2 × STAGE_LEN_MS`.

`Class` itself is computed by the crate's classifier: `distance_transform` runs a
2-pass chamfer transform (distance to the nearest opposite-colour pixel, saturating
at 3, panel edge treated as a boundary), and `classify` buckets each pixel into
`Interior` (distance ≥ 3), `Isolated` (distance 1 and a local maximum), or `Edge`
(otherwise).

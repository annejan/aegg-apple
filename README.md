# aegg-apple

Bad Apple!! on the CyberAegg badge — video on the e-paper panel, audio out of the
speaker.

Target hardware is the BornHack 2026 badge: nRF52840, 152×152 SSD1675 e-paper,
external QSPI flash, PWM-driven speaker. Drivers and build patterns are borrowed
from [`bornhack-firmware-2026`](https://github.com/bornhack/bornhack-firmware-2026).

## Status

Encoder works and round-trips. No firmware yet.

## Hardware constraints

These set every design decision, so they're worth stating up front:

- **The panel is the bottleneck.** A whole-panel partial refresh measures
  ~500 ms at the stock LUT speed. `EPD_LUT_SPEED_MIN` clamps the fast end at
  30, i.e. roughly 150–200 ms, so the ceiling is about **5 fps** and the
  realistic figure is lower. Assets are encoded at 4 fps and the player drops
  frames rather than letting audio drift.
- **The partial-refresh engine must not promote to a full refresh.** Stock
  `full_after_screens = 3` triggers a full drive once cumulative changed
  pixels hit three screens' worth — several seconds of waveform, every couple
  of seconds of video.
- **There is no DAC.** Audio is a piezo buzzer on P0_13 driven by PWM0.
  Sample playback means duty-modulating it from EasyDMA; the piezo's
  resonance, not the bit depth, is what limits how it sounds.
- **QSPI is 2 MiB.** The stock firmware splits it into a 1 MiB `ekv` key-value
  store plus a 1 MiB FAT12 partition. This firmware has no pet game and no
  settings to persist, so FAT12 gets the whole 2 MiB.

## Asset format

`tools/encode.py` produces two files for the badge's FAT12 volume.

**`BADAPPLE.VID`** — 152×152 1bpp frames, bit-run-length coded, intra-only,
preceded by a frame offset table so the player can jump to whichever frame the
audio clock says is current. Runs under 128 take one byte, longer runs take
two.

Coding the bit runs rather than byte runs is what makes this fit, and
XOR-delta between frames turned out to be a trap — it wins on only 65 of 876
frames, because the delta breaks up the long uniform runs that Bad Apple
compresses so well in the first place. Measured, whole track:

| | 2 fps | 4 fps | 6 fps |
|---|---|---|---|
| byte-RLE | 382 K | 747 K | 1086 K |
| **bit-RLE** | **141 K** | **284 K** | **420 K** |
| zlib, XOR-delta | 190 K | 361 K | 515 K |

Dithering roughly doubles the size and the panel cannot hold the detail at
speed, so frames are plain-thresholded.

**`BADAPPLE.SND`** — 4-bit IMA ADPCM, mono, in self-contained blocks that each
restart the predictor so playback can seek.

At 4 fps and 8 kHz the pair comes to **1152 KiB of the 2048 KiB** available.

## Building the assets

The video is not committed. Fetch and encode:

```sh
yt-dlp -f 'bv*[height<=480]+ba/b[height<=480]' -o assets/badapple.%\(ext\)s \
  'https://www.youtube.com/watch?v=FtutLA63Cp8'
python3 tools/encode.py assets/badapple.webm --fps 4 --rate 8000
```

Outputs land in `assets/to-badge/`, to be copied onto the badge's USB volume.

`tools/probe_sizes.py` regenerates the codec comparison above.

## License

MIT

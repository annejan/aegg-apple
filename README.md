# aegg-apple

Bad Apple!! on the CyberAegg badge — video on the e-paper panel, audio out of the
speaker.

Target hardware is the BornHack 2026 badge: nRF52840, 152×152 SSD1675 e-paper,
external QSPI flash, PWM-driven speaker. Drivers and build patterns are borrowed
from [`bornhack-firmware-2026`](https://github.com/bornhack/bornhack-firmware-2026).

## Status

Plays, with sound, on real hardware. Measured **~1 fps** at the default
waveform strength.

## Hardware constraints

These set every design decision, so they're worth stating up front. The
numbers are measured on a badge, not estimated.

- **The panel is the bottleneck.** A partial refresh takes ~1020 ms at the
  default waveform strength and ~477 ms at the lightest one that still draws.
  Roughly 400 ms of that is fixed cost that is not waveform time, so pushing
  the waveform lighter buys less than it looks like it should. Assets are
  encoded at 4 fps and the player drops frames to stay with the audio.
- **Sharpness and frame rate trade against each other.** A lighter waveform
  refreshes faster but drives each pixel less far, so the image softens and
  the previous frame shows through. Fire cycles the setting at runtime
  precisely because there is no single right answer; 30 is the default.
- **No full refreshes, ever.** The full waveform does not complete on this
  panel — it overruns its budget and leaves BUSY asserted, and every partial
  after it then inherits a busy controller and stalls. Playback never issues
  one, not even at boot. The ghosting this leaves is part of the look.
- **The red row must be zeroed.** The panel is tri-colour, and black and red
  pigment both respond to drive. A delta waveform with the red row live pulls
  red particles up on pixels only ever meant to be black or white, and the
  picture goes pink.
- **There is no DAC.** Audio is a piezo buzzer on P0_13 driven by PWM0.
  Samples are played by duty-modulating a 54 kHz carrier, each sample held for
  nine periods. Putting one PWM period per sample instead makes the carrier
  *be* the sample rate, which is a buzz, not music.
- **The asset budget is ~1000 KiB.** QSPI is 2 MiB, split into a 1 MiB `ekv`
  key-value store and the 1 MiB FAT12 partition exposed over USB. Only the
  FAT12 half is used; `ekv` is left alone so a stock badge stays recoverable.

## Running it

Assets go on the badge's USB volume (it appears as a small removable drive);
the firmware reads `BADAPPLE.VID` and `BADAPPLE.SND` from there. The pet
game's sprites and the assets do not both fit, so the volume has to be
cleared first.

```sh
cargo build --release
# with a debug probe:
probe-rs download --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/aegg-apple
# without one, in DFU mode (hold Execute, tap reset), then POWER-CYCLE after:
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/aegg-apple fw.bin
dfu-util -D fw.bin
```

`dfu-util -R` does not hand off cleanly on this bootloader — power-cycle by
hand. The bootloader owns the low 64 KB and is never written, so DFU cannot
brick the badge.

Press **Fire** during playback to cycle the waveform strength.

### Debugging without a probe

The firmware brings up USB CDC serial before anything that can fail, and logs
the parsed headers, the detected panel variant, the waveform frame counts and
per-frame refresh timings. `cat /dev/ttyACM0`. A panic lights the red LED
rather than trapping silently, so a dead badge is distinguishable from a
stalled one.

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

Audio is shaped for the transducer before it is encoded. The piezo radiates
essentially nothing below a few hundred Hz, so the signal is bandpassed to
500–3400 Hz, then compressed and limited into the available headroom. Loudness
has to come from compression: the limiter caps peaks, so raising the gain
alone changes nothing above about 28% RMS.

Doing this at encode time rather than in firmware matters. Applying gain on the
badge can only hard-clip, and clipping a 4-bit ADPCM stream sounds like grit
rather than volume — that was the first version and it sounded bad. Measured
over the whole track, the shaped version reaches 37% RMS against 27%, with
clipping down at 0.2%.

It is still 4-bit ADPCM through a piezo, at about 14 dB SNR. Better, not good.

At 2 fps and 7500 Hz the pair comes to **951 KiB**, against the ~1000 KiB the
FAT12 volume holds.

The split between them is worth noting: the panel only manages about 1 fps, so
encoding video at 2 fps rather than 4 costs nothing visible and frees ~150 KiB,
which is spent on the audio sample rate instead — the one place the extra bytes
are actually audible.

## Building the assets

The video is not committed. Fetch and encode:

```sh
yt-dlp -f 'bv*[height<=480]+ba/b[height<=480]' -o assets/badapple.%\(ext\)s \
  'https://www.youtube.com/watch?v=FtutLA63Cp8'
python3 tools/encode.py assets/badapple.webm --fps 4 --rate 6000
```

Outputs land in `assets/to-badge/`, to be copied onto the badge's USB volume.

`tools/probe_sizes.py` regenerates the codec comparison above.

## License

MIT

# aegg-apple

Bad Apple!! on the CyberAegg badge — video on the e-paper panel, audio out of the
speaker.

Target hardware is the BornHack 2026 badge: nRF52840, 152×152 SSD1675 e-paper,
external QSPI flash, PWM-driven speaker. Drivers and build patterns are borrowed
from [`bornhack-firmware-2026`](https://github.com/bornhack/bornhack-firmware-2026).

## Status

Early. Nothing runs yet.

## Plan

1. **Encoder** (host side): decimate the source video to the panel's 152×152
   1bpp framebuffer at the highest frame rate the panel's custom LUT can
   actually sustain, then delta+RLE compress the frame stream. Audio is
   downsampled to 8-bit PCM for PWM playback.
2. **Asset image**: frames and PCM are packed into a single blob flashed to the
   external QSPI flash, not into the 1 MB internal flash.
3. **Player** (firmware): a timer-driven loop that streams frames from QSPI to
   the panel while a PWM sequence plays the PCM, both slaved to one clock so
   they stay in sync.

The e-paper refresh rate is the hard constraint — see `LUT.md` in the firmware
repo for the custom-waveform path that makes a fast, light refresh possible.

## Source material

The video is not committed. Fetch it with:

```sh
yt-dlp -f 'bv*[height<=480]+ba/b[height<=480]' -o assets/badapple.%\(ext\)s \
  'https://www.youtube.com/watch?v=FtutLA63Cp8'
```

## License

MIT

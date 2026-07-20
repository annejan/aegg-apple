#!/usr/bin/env python3
"""Size probe for the asset budget.

Decodes the source video to 152x152 1bpp frames at a few candidate frame
rates, compresses each with the delta+RLE scheme the player will use, and
reports the resulting byte count. Also sizes the audio track as 4-bit
IMA-ADPCM at a few sample rates.

Nothing here is the real encoder -- it exists to answer "does it fit"
before any firmware gets written.
"""

import subprocess
import sys
from pathlib import Path

W = H = 152
STRIDE = W // 8
FRAME_BYTES = STRIDE * H

VIDEO_FPS = [2, 3, 4, 5, 6]
AUDIO_RATES = [4000, 6000, 8000, 11025]


def decode_frames(src: Path, fps: int) -> list[bytes]:
    """Return 1bpp MSB-first frames, one bytes object per frame."""
    cmd = [
        "ffmpeg", "-v", "error", "-i", str(src),
        "-vf", f"fps={fps},scale={W}:{H}:force_original_aspect_ratio=decrease,"
               f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color=black,format=gray",
        "-f", "rawvideo", "-pix_fmt", "gray", "-",
    ]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    px_per_frame = W * H
    frames = []
    for off in range(0, len(raw) - px_per_frame + 1, px_per_frame):
        gray = raw[off:off + px_per_frame]
        buf = bytearray(FRAME_BYTES)
        for y in range(H):
            row = y * W
            for x in range(W):
                if gray[row + x] >= 128:  # white
                    buf[(x >> 3) + STRIDE * y] |= 0x80 >> (x & 7)
        frames.append(bytes(buf))
    return frames


def rle(data: bytes) -> int:
    """Byte count of a simple run-length encoding: (count, value) pairs,
    runs capped at 255. Returns size only -- this is a probe."""
    if not data:
        return 0
    size = 0
    run = 1
    prev = data[0]
    for b in data[1:]:
        if b == prev and run < 255:
            run += 1
        else:
            size += 2
            prev, run = b, 1
    return size + 2


def compress_stream(frames: list[bytes]) -> tuple[int, int]:
    """XOR each frame against the previous, RLE the result. Returns
    (total bytes, count of frames stored whole because the delta was
    bigger than the keyframe)."""
    total = 0
    keyframes = 0
    prev = bytes(FRAME_BYTES)
    for f in frames:
        delta = bytes(a ^ b for a, b in zip(f, prev))
        d_size = rle(delta)
        k_size = rle(f)
        if k_size < d_size:
            total += k_size + 1
            keyframes += 1
        else:
            total += d_size + 1
        prev = f
    return total, keyframes


# --- IMA ADPCM ------------------------------------------------------------

STEP_TABLE = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37,
    41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173,
    190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
    724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
    6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289,
    16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
]
INDEX_TABLE = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8]


def adpcm_encode(pcm: bytes) -> bytes:
    """Encode signed 16-bit LE PCM to 4-bit IMA ADPCM, two nibbles per byte."""
    out = bytearray()
    pred = 0
    index = 0
    pending = None
    for i in range(0, len(pcm) - 1, 2):
        sample = int.from_bytes(pcm[i:i + 2], "little", signed=True)
        step = STEP_TABLE[index]
        diff = sample - pred
        code = 0
        if diff < 0:
            code = 8
            diff = -diff
        delta = step >> 3
        for bit in (4, 2, 1):
            if diff >= step:
                code |= bit
                diff -= step
                delta += step
            step >>= 1
        pred += -delta if code & 8 else delta
        pred = max(-32768, min(32767, pred))
        index = max(0, min(88, index + INDEX_TABLE[code]))
        if pending is None:
            pending = code
        else:
            out.append(pending | (code << 4))
            pending = None
    if pending is not None:
        out.append(pending)
    return bytes(out)


def audio_pcm(src: Path, rate: int) -> bytes:
    cmd = [
        "ffmpeg", "-v", "error", "-i", str(src),
        "-ac", "1", "-ar", str(rate), "-f", "s16le", "-",
    ]
    return subprocess.run(cmd, capture_output=True, check=True).stdout


def main() -> None:
    src = Path(sys.argv[1])
    print(f"source: {src}\n")

    print("video (152x152 1bpp, XOR delta + RLE)")
    print(f"{'fps':>4}  {'frames':>7}  {'bytes':>10}  {'KiB':>7}  {'keyframes':>9}")
    for fps in VIDEO_FPS:
        frames = decode_frames(src, fps)
        total, keys = compress_stream(frames)
        print(f"{fps:>4}  {len(frames):>7}  {total:>10}  {total/1024:>7.1f}  {keys:>9}")

    print("\naudio (4-bit IMA ADPCM, mono)")
    print(f"{'rate':>6}  {'bytes':>10}  {'KiB':>7}")
    for rate in AUDIO_RATES:
        enc = adpcm_encode(audio_pcm(src, rate))
        print(f"{rate:>6}  {len(enc):>10}  {len(enc)/1024:>7.1f}")


if __name__ == "__main__":
    main()

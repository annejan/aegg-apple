#!/usr/bin/env python3
"""Encode a video into the two asset files the badge plays.

    BADAPPLE.VID   152x152 1bpp frames, bit-run-length coded
    BADAPPLE.SND   4-bit IMA ADPCM, mono, block-framed

Both are copied onto the badge's FAT12 volume over USB mass storage.

Frames are coded intra-only. XOR-delta was measured against this and lost
on 811 of 876 frames -- the delta breaks up exactly the long uniform runs
that make Bad Apple compress, so it costs more than it saves.
"""

import argparse
import struct
import subprocess
import sys
from pathlib import Path

W = H = 152
STRIDE = W // 8
FRAME_BYTES = STRIDE * H

VID_MAGIC = b"AEGGVID1"
SND_MAGIC = b"AEGGSND1"

# Samples per ADPCM block. Each block restarts the predictor so the player
# can seek without decoding everything before it. The first sample lives in
# the header, leaving 1016 nibbles: 4 + 508 = a round 512 bytes per block.
BLOCK_SAMPLES = 1017


# --- video ----------------------------------------------------------------

def decode_frames(src: Path, fps: int, invert: bool) -> list[bytes]:
    """Scale to 152x152, threshold to 1bpp, MSB-first, row-major.

    A set bit means white, matching the SSD1675 black-plane convention.
    """
    cmd = [
        "ffmpeg", "-v", "error", "-i", str(src),
        "-vf", f"fps={fps},scale={W}:{H}:force_original_aspect_ratio=decrease,"
               f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color=black,format=gray",
        "-f", "rawvideo", "-pix_fmt", "gray", "-",
    ]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    px = W * H
    frames = []
    for off in range(0, len(raw) - px + 1, px):
        gray = raw[off:off + px]
        buf = bytearray(FRAME_BYTES)
        for y in range(H):
            row = y * W
            base = STRIDE * y
            for x in range(W):
                lit = gray[row + x] >= 128
                if lit != invert:
                    buf[base + (x >> 3)] |= 0x80 >> (x & 7)
        frames.append(bytes(buf))
    return frames


def bit_rle(data: bytes) -> bytes:
    """Run-length code the bit stream, MSB first.

    Output is the value of the first bit (0x00 or 0x01) followed by run
    lengths. A run under 128 is one byte; longer runs are two bytes,
    0x80|hi7 then lo8, encoding up to 32767 -- more than the 23104 bits
    in a frame, so a run always fits.
    """
    out = bytearray()
    bits = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)

    cur = bits[0]
    out.append(cur)
    run = 1
    for b in bits[1:]:
        if b == cur:
            run += 1
            continue
        _emit_run(out, run)
        cur, run = b, 1
    _emit_run(out, run)
    return bytes(out)


def _emit_run(out: bytearray, run: int) -> None:
    if run < 128:
        out.append(run)
    else:
        out.append(0x80 | (run >> 8))
        out.append(run & 0xFF)


def bit_rle_decode(enc: bytes, nbytes: int) -> bytes:
    """Reference decoder -- used to verify the encoder round-trips."""
    out = bytearray(nbytes)
    cur = enc[0]
    pos = 0
    i = 1
    while i < len(enc):
        run = enc[i]
        i += 1
        if run & 0x80:
            run = ((run & 0x7F) << 8) | enc[i]
            i += 1
        if cur:
            for k in range(pos, pos + run):
                out[k >> 3] |= 0x80 >> (k & 7)
        pos += run
        cur ^= 1
    return bytes(out)


def write_vid(path: Path, frames: list[bytes], fps: int) -> None:
    """Header, then a frame offset table, then the coded frames.

    The offset table lets the player jump straight to whichever frame the
    audio clock says is current, so a slow refresh drops frames instead of
    drifting out of sync.
    """
    coded = [bit_rle(f) for f in frames]

    for original, enc in zip(frames, coded):
        if bit_rle_decode(enc, FRAME_BYTES) != original:
            sys.exit("bit-RLE round-trip failed")

    header_len = len(VID_MAGIC) + 12
    table_len = 4 * (len(coded) + 1)
    base = header_len + table_len

    offsets = []
    pos = base
    for enc in coded:
        offsets.append(pos)
        pos += len(enc)
    offsets.append(pos)

    with path.open("wb") as fh:
        fh.write(VID_MAGIC)
        fh.write(struct.pack("<HHHHI", W, H, fps, 0, len(coded)))
        for off in offsets:
            fh.write(struct.pack("<I", off))
        for enc in coded:
            fh.write(enc)


# --- audio ----------------------------------------------------------------

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


def read_pcm(src: Path, rate: int, gain: float) -> list[int]:
    cmd = ["ffmpeg", "-v", "error", "-i", str(src), "-ac", "1",
           "-ar", str(rate)]
    if gain != 1.0:
        cmd += ["-af", f"volume={gain}"]
    cmd += ["-f", "s16le", "-"]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    return list(struct.unpack(f"<{len(raw)//2}h", raw[:len(raw)//2*2]))


def adpcm_block(samples: list[int]) -> bytes:
    """Encode one block. The first sample is stored verbatim as the
    starting predictor, so the block decodes standalone."""
    pred = samples[0]
    index = 0
    out = bytearray(struct.pack("<hBB", pred, index, 0))

    nibbles = []
    for sample in samples[1:]:
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
        pred = max(-32768, min(32767, pred - delta if code & 8 else pred + delta))
        index = max(0, min(88, index + INDEX_TABLE[code]))
        nibbles.append(code)

    if len(nibbles) % 2:
        nibbles.append(0)
    for i in range(0, len(nibbles), 2):
        out.append(nibbles[i] | (nibbles[i + 1] << 4))
    return bytes(out)


def write_snd(path: Path, samples: list[int], rate: int) -> None:
    blocks = [samples[i:i + BLOCK_SAMPLES]
              for i in range(0, len(samples), BLOCK_SAMPLES)]
    if blocks and len(blocks[-1]) < 2:
        blocks.pop()

    encoded = [adpcm_block(b) for b in blocks]
    with path.open("wb") as fh:
        fh.write(SND_MAGIC)
        fh.write(struct.pack("<IIHH", rate, len(samples), BLOCK_SAMPLES,
                             len(encoded[0]) if encoded else 0))
        for blk in encoded:
            fh.write(blk)


# --- main -----------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("source", type=Path)
    ap.add_argument("-o", "--outdir", type=Path, default=Path("assets/to-badge"))
    ap.add_argument("--fps", type=int, default=4)
    ap.add_argument("--rate", type=int, default=8000)
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--invert", action="store_true",
                    help="swap black and white")
    args = ap.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)

    frames = decode_frames(args.source, args.fps, args.invert)
    vid = args.outdir / "BADAPPLE.VID"
    write_vid(vid, frames, args.fps)
    print(f"{vid}  {len(frames)} frames @ {args.fps}fps  "
          f"{vid.stat().st_size/1024:.1f} KiB")

    samples = read_pcm(args.source, args.rate, args.gain)
    snd = args.outdir / "BADAPPLE.SND"
    write_snd(snd, samples, args.rate)
    print(f"{snd}  {len(samples)} samples @ {args.rate}Hz  "
          f"{snd.stat().st_size/1024:.1f} KiB")

    # The badge's FAT12 volume is the second MiB of the 2 MiB QSPI part; the
    # first MiB belongs to the ekv key-value store and is left alone.
    total = vid.stat().st_size + snd.stat().st_size
    budget = 1000 * 1024
    print(f"total {total/1024:.1f} KiB of ~1000 KiB usable"
          f"  ({100*total/budget:.0f}%)")
    if total > budget:
        sys.exit("assets do not fit the FAT12 volume")


if __name__ == "__main__":
    main()

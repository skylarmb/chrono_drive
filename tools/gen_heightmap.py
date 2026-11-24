#!/usr/bin/env python3
"""Generate data/maps/test_track.bmp: a flat strip along X with rolling hills on both sides.

The image is 8-bit grayscale (black = min_height, white = max_height in main.cpp).
The image spans the full map, so with a 60 m map one pixel is 60/N m.
Usage: tools/gen_heightmap.py [out.bmp]
"""
import math
import struct
import sys

N = 400            # image size [px]
MAP = 60.0         # map edge length [m], must match main.cpp
STRIP = 4.0        # half-width of the flat strip [m]
BLEND = 6.0        # distance over which hills rise from the strip [m]


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def height(x, y):
    """Normalized height in [0, 1] at map position (x, y) [m]."""
    hills = 0.5 + 0.25 * math.sin(x / 5.0) * math.cos(y / 4.0) + 0.25 * math.sin((x + y) / 9.0)
    return hills * smoothstep((abs(y) - STRIP) / BLEND)


def write_bmp(path, rows):
    """Write bottom-up 8-bit paletted BMP; rows[0] is the bottom row."""
    stride = (N + 3) & ~3
    pixels = b"".join(bytes(r) + b"\0" * (stride - N) for r in rows)
    palette = b"".join(bytes((i, i, i, 0)) for i in range(256))
    offset = 14 + 40 + len(palette)
    header = b"BM" + struct.pack("<IHHI", offset + len(pixels), 0, 0, offset)
    info = struct.pack("<IiiHHIIiiII", 40, N, N, 1, 8, 0, len(pixels), 2835, 2835, 256, 0)
    with open(path, "wb") as f:
        f.write(header + info + palette + pixels)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "data/maps/test_track.bmp"
    # Pixel centers span [-MAP/2, MAP/2]. Columns map to X, rows to Y.
    coord = lambda i: (i + 0.5) / N * MAP - MAP / 2
    rows = [[round(255 * height(coord(c), coord(r))) for c in range(N)] for r in range(N)]
    write_bmp(out, rows)


if __name__ == "__main__":
    main()

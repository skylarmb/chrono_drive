#!/usr/bin/env python3
"""Generate data/maps/test_track.bmp: the terrain of the test course.

A flat strip runs along X at height 0, with rolling hills on both sides. Two
terrain sections sit on the strip near the end of the course (main.cpp places
the rigid obstacles before them):
- cross-axle ditches: dips under the left and right wheel tracks, in turn
- side slope: the strip tilts sideways, then levels out again

The image is 8-bit grayscale. Black is MIN_HEIGHT and white is MAX_HEIGHT.
The image spans the full map. MAP_X, MAP_Y, MIN_HEIGHT and MAX_HEIGHT must match main.cpp.
Usage: tools/gen_heightmap.py [out.bmp]
"""
import math
import struct
import sys

MAP_X, MAP_Y = 140.0, 40.0          # map size [m]
MIN_HEIGHT, MAX_HEIGHT = -0.5, 2.0  # heights of black and white [m]
PX_PER_M = 8                        # image resolution

STRIP = 4.0          # half-width of the flat strip [m]
BLEND = 6.0          # distance over which the hills rise from the strip [m]
HILL_HEIGHT = 1.8    # [m]

# Cross-axle ditches: (x center [m], side +1 = left / -1 = right)
DITCHES = [(34, 1), (37, -1), (40, 1), (43, -1)]
DITCH_DEPTH = 0.30   # [m]
DITCH_LENGTH = 2.4   # along X [m]
DITCH_WIDTH = 1.6    # across the wheel track [m]
WHEEL_TRACK_Y = 0.9  # half track of the test vehicle [m]

# Side slope: the strip tilts by SLOPE_DEG between these X positions (with ramps in and out)
SLOPE_START, SLOPE_END, SLOPE_RAMP = 50.0, 64.0, 4.0
SLOPE_DEG = 12.0


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def bump(t):
    """Smooth dip profile on [-1, 1]: 1 at the center, 0 at the edges."""
    return 0.5 * (1 + math.cos(math.pi * t)) if abs(t) < 1 else 0.0


def height(x, y):
    """Height [m] at map position (x, y) [m]."""
    hills = HILL_HEIGHT * (0.5 + 0.25 * math.sin(x / 5.0) * math.cos(y / 4.0) + 0.25 * math.sin((x + y) / 9.0))
    h = hills * smoothstep((abs(y) - STRIP) / BLEND)

    for xc, side in DITCHES:
        h -= DITCH_DEPTH * bump((x - xc) / (DITCH_LENGTH / 2)) * bump((y - side * WHEEL_TRACK_Y) / (DITCH_WIDTH / 2))

    # Side slope: full tilt inside the section, smooth ramps at both ends, only on the strip.
    tilt = smoothstep((x - SLOPE_START) / SLOPE_RAMP) * (1 - smoothstep((x - SLOPE_END) / SLOPE_RAMP))
    h += tilt * math.tan(math.radians(SLOPE_DEG)) * max(-STRIP, min(STRIP, y))
    return h


def write_bmp(path, nx, ny, rows):
    """Write a bottom-up 8-bit paletted BMP. rows[0] is the bottom row."""
    stride = (nx + 3) & ~3
    pixels = b"".join(bytes(r) + b"\0" * (stride - nx) for r in rows)
    palette = b"".join(bytes((i, i, i, 0)) for i in range(256))
    offset = 14 + 40 + len(palette)
    header = b"BM" + struct.pack("<IHHI", offset + len(pixels), 0, 0, offset)
    info = struct.pack("<IiiHHIIiiII", 40, nx, ny, 1, 8, 0, len(pixels), 2835, 2835, 256, 0)
    with open(path, "wb") as f:
        f.write(header + info + palette + pixels)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "data/maps/test_track.bmp"
    nx, ny = int(MAP_X * PX_PER_M), int(MAP_Y * PX_PER_M)
    # Pixel centers span the map. Columns map to X, rows to Y.
    gray = lambda h: max(0, min(255, round(255 * (h - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT))))
    rows = [[gray(height((c + 0.5) / nx * MAP_X - MAP_X / 2, (r + 0.5) / ny * MAP_Y - MAP_Y / 2))
             for c in range(nx)] for r in range(ny)]
    write_bmp(out, nx, ny, rows)


if __name__ == "__main__":
    main()

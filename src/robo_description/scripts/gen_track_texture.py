#!/usr/bin/env python3
"""Generate a track ground texture: light ground with a black line track.

The track is a single continuous loop-ish course on an 8 m x 8 m ground
plane, featuring (in driving order from the start):

    1. an entry straight,
    2. an S-curve (left then right),
    3. a long straight,
    4. a 180 deg U-turn,
    5. a return straight back toward the start area.

The same waypoint path is used by ``track_path.py`` so traffic-light
placement in the world matches the painted line exactly.
"""
import os

from PIL import Image, ImageDraw

from track_path import GROUND_M, LINE_WIDTH_M, sample_path

SIZE = 2048                       # px (texture resolution)
PX_PER_M = SIZE / GROUND_M        # px per metre
LINE_WIDTH_PX = max(1, int(LINE_WIDTH_M * PX_PER_M))


def m_to_px(mx, my):
    return int(mx * PX_PER_M), int(my * PX_PER_M)


def main():
    img = Image.new("RGB", (SIZE, SIZE), (220, 220, 220))
    draw = ImageDraw.Draw(img)

    pts = [m_to_px(x, y) for (x, y) in sample_path(n=1500)]
    # Single continuous polyline with rounded joints; round caps at the ends.
    draw.line(pts, fill=(10, 10, 10), width=LINE_WIDTH_PX, joint="curve")
    r = LINE_WIDTH_PX // 2
    for px, py in (pts[0], pts[-1]):
        draw.ellipse([px - r, py - r, px + r, py + r], fill=(10, 10, 10))

    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(
        out_dir, "..", "models", "track_ground",
        "materials", "textures", "track.png",
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    img.save(out_path)
    print(f"Saved track texture ({SIZE}x{SIZE}) to {out_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Shared definition of the robo404+ simulation track centerline.

Both the texture generator and the world generator import this module so
the painted line and the prop / traffic-light placement stay in sync.

Coordinate convention (metres), matching the texture image:
  * origin (0,0) is the top-left of the 8x8 m ground image,
  * +X is to the right, +Y is downward (image row direction).

The world maps this image onto the ground plane centered at the world
origin, so world_xy = image_xy - GROUND_M/2  (see gen_world.py).

The course, in driving order from START:
  Stadium / "playground" loop matching the real-world track:
  1. top straight         (left -> right)
  2. right U-turn         (180 deg, heading +X -> -X)
  3. bottom straight      (right -> left)
  4. left U-turn          (180 deg, heading -X -> +X, closes the loop)
"""
import math

GROUND_M = 4.0            # ground plane is GROUND_M x GROUND_M metres
LINE_WIDTH_M = 0.04       # painted line width (~real tape width at this scale)


# The course is built by integrating heading over a sequence of primitives.
# Each primitive is (length_m, total_turn_rad): turn=0 is straight, +turn is a
# left turn (CCW in image frame), -turn is a right turn. Heading is continuous,
# so every junction is tangent-continuous and the loop closes smoothly.
#
# True real-scale 400 m-track shape: ~1 m straights with the standard
# straight/radius ratio (84.39 / 36.5 ~= 2.31) -> R_TURN ~= 0.43 m.
#   STRAIGHT = 1.0 m, R_TURN = 0.43 m
#   overall track ~= (STRAIGHT + 2*R_TURN) x (2*R_TURN) = 1.86 x 0.86 m
# Placed on a 4x4 m ground, roughly centered.
R_TURN = 0.43
STRAIGHT = 1.0
START_XY = (1.4, 1.6)     # on the top straight, near its left end
START_HEADING = 0.0       # +X (driving to the right)

# S-curve inserted into the bottom straight: a symmetric left-right-right-left
# bump (R_S radius, S_ANGLE per bend) that returns to the same heading AND zero
# net lateral offset, so the stadium loop still closes. Short pads either side
# make the bottom run span the same width as the top straight.
R_S = 0.25                # S-curve bend radius (tighter than the U-turns)
S_ANGLE = math.pi / 3     # 60 deg per bend
_S_SPAN = 0.866           # horizontal distance the S-bump covers (measured)
_S_PAD = (STRAIGHT - _S_SPAN) / 2.0

PRIMITIVES = [
    (STRAIGHT, 0.0),                 # 1. top straight (heading +X)
    (math.pi * R_TURN, +math.pi),    # 2. right U-turn (180 deg, curves down) -> -X
    # 3. bottom run with an S-curve in the middle (net 0 turn, net 0 offset)
    (_S_PAD, 0.0),
    (S_ANGLE * R_S, +S_ANGLE),
    (S_ANGLE * R_S, -S_ANGLE),
    (S_ANGLE * R_S, -S_ANGLE),
    (S_ANGLE * R_S, +S_ANGLE),
    (_S_PAD, 0.0),
    (math.pi * R_TURN, +math.pi),    # 4. left U-turn (180 deg, curves up) -> closes loop
]
STEP_M = 0.02             # integration step


def path_segments():
    """Return the path as one continuous segment list of (x,y) points."""
    x, y = START_XY
    h = START_HEADING
    pts = [(x, y)]
    for length, turn in PRIMITIVES:
        n = max(1, int(round(length / STEP_M)))
        dh = turn / n
        ds = length / n
        for _ in range(n):
            h += dh
            x += ds * math.cos(h)
            y += ds * math.sin(h)
            pts.append((x, y))
    return [pts]


def sample_path(n=1000):
    """Return a single flat list of (x,y) points sampling the whole path."""
    segs = path_segments()
    flat = []
    for seg in segs:
        if flat and seg and _close(flat[-1], seg[0]):
            flat.extend(seg[1:])
        else:
            flat.extend(seg)

    # Resample to roughly n evenly distributed points by arc length.
    if len(flat) <= 2:
        return flat
    cum = [0.0]
    for i in range(1, len(flat)):
        cum.append(cum[-1] + _dist(flat[i - 1], flat[i]))
    total = cum[-1]
    out = []
    for k in range(n + 1):
        target = total * k / n
        out.append(_interp_at(flat, cum, target))
    return out


def start_pose():
    """(x, y, yaw) world-frame spawn pose for the robot at the path start."""
    wx, wy = image_to_world(*START_XY)
    # Image heading +X with image-Y-down maps to world heading +X with Y-up,
    # so the world yaw is the negated image heading.
    yaw = -START_HEADING
    return wx, wy, yaw


def image_to_world(ix, iy):
    """Map image-frame (x,y) to world-frame (x,y).

    Image +Y is downward; world +Y is up, so flip Y about the ground center.
    """
    wx = ix - GROUND_M / 2.0
    wy = (GROUND_M / 2.0) - iy
    return wx, wy


def _close(a, b, eps=1e-6):
    return abs(a[0] - b[0]) < eps and abs(a[1] - b[1]) < eps


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _interp_at(flat, cum, target):
    import bisect
    i = bisect.bisect_left(cum, target)
    if i <= 0:
        return flat[0]
    if i >= len(flat):
        return flat[-1]
    seg_len = cum[i] - cum[i - 1]
    if seg_len <= 0:
        return flat[i]
    t = (target - cum[i - 1]) / seg_len
    ax, ay = flat[i - 1]
    bx, by = flat[i]
    return (ax + (bx - ax) * t, ay + (by - ay) * t)

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

GROUND_M = 6.0            # ground plane is GROUND_M x GROUND_M metres
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
# Irregular organic circuit, defined by hand-placed waypoints (image frame,
# metres on the 6x6 ground) and Chaikin corner-cutting smoothing. The waypoints
# are spaced so the smoothed loop's tightest bend stays >= ~0.45 m radius, which
# the line-follower can track. Asymmetric on purpose -- varied sweeps, a kink on
# the lower-left, no symmetry.
WAYPOINTS = [
    (1.6, 1.1), (3.2, 0.9), (4.6, 1.6), (4.3, 3.0), (5.0, 4.0),
    (3.8, 4.7), (2.4, 4.4), (2.7, 3.2), (1.4, 3.4), (0.9, 2.1),
]
CHAIKIN_PASSES = 4

STEP_M = 0.02             # (re)sampling step, kept for downstream callers


def _chaikin_closed(pts, passes):
    """Closed Chaikin corner-cutting: smooths a polygon into a rounded loop."""
    for _ in range(passes):
        n = len(pts)
        out = []
        for i in range(n):
            p = pts[i]
            q = pts[(i + 1) % n]
            out.append((0.75 * p[0] + 0.25 * q[0], 0.75 * p[1] + 0.25 * q[1]))
            out.append((0.25 * p[0] + 0.75 * q[0], 0.25 * p[1] + 0.75 * q[1]))
        pts = out
    return pts


def path_segments():
    """Return the path as one continuous closed segment of (x,y) points."""
    loop = _chaikin_closed(WAYPOINTS, CHAIKIN_PASSES)
    loop = loop + [loop[0]]            # explicitly close the loop
    return [loop]


# START is the first point of the smoothed loop, heading toward the next point.
_loop0 = path_segments()[0]
START_XY = _loop0[0]
START_HEADING = math.atan2(_loop0[1][1] - _loop0[0][1],
                           _loop0[1][0] - _loop0[0][0])


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

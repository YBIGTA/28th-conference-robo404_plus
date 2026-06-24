"""Geometry sanity checks for the auto-labeller (runnable without ROS)."""
import math

import numpy as np

from yolo_dataset.projection import (
    compose, invert_tf, make_tf, project_object, quat_to_rot, rpy_to_rot,
)

# 640x480 pinhole, 60deg hfov -> fx ~ 554
W, H = 640, 480
FX = W / (2 * math.tan(math.radians(60) / 2))
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def _cam_looking_down_x():
    """world->camera transform for a camera at origin whose optical z = world +x.

    Rows of R are the optical axes in world coords:
      x-right = world -y, y-down = world -z, z-forward = world +x
    so p_cam = R @ p_world directly (this IS T_cam_world)."""
    R = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)
    return make_tf([0, 0, 0], R)


def test_object_dead_ahead_is_centered():
    T_cam_world = _cam_looking_down_x()
    out = project_object(K, T_cam_world, [3.0, 0.0, 0.0], IDENTITY_Q, [0.05, 0.05, 0.15], W, H)
    assert out is not None
    cx, cy, bw, bh = out["bbox_xywh_norm"]
    assert abs(cx - 0.5) < 0.02 and abs(cy - 0.5) < 0.02   # centered
    assert abs(out["distance_m"] - 3.0) < 1e-3
    assert 0 < bw < 0.3 and 0 < bh < 0.5


def test_object_behind_camera_is_skipped():
    T_cam_world = _cam_looking_down_x()
    assert project_object(K, T_cam_world, [-3.0, 0, 0], IDENTITY_Q, [0.1, 0.1, 0.1], W, H) is None


def test_object_to_the_left_has_smaller_cx():
    T_cam_world = _cam_looking_down_x()
    left = project_object(K, T_cam_world, [3.0, 0.6, 0.0], IDENTITY_Q, [0.05, 0.05, 0.15], W, H)
    # world +y maps to optical -x (image left) -> cx < 0.5
    assert left is not None and left["bbox_xywh_norm"][0] < 0.5


def test_closer_object_is_bigger():
    T_cam_world = _cam_looking_down_x()
    near = project_object(K, T_cam_world, [1.5, 0, 0], IDENTITY_Q, [0.1, 0.1, 0.1], W, H)
    far = project_object(K, T_cam_world, [4.0, 0, 0], IDENTITY_Q, [0.1, 0.1, 0.1], W, H)
    assert near["bbox_xywh_norm"][2] > far["bbox_xywh_norm"][2]
    assert near["distance_m"] < far["distance_m"]


def test_rpy_and_quat_agree_on_yaw():
    r1 = rpy_to_rot(0, 0, math.pi / 2)
    r2 = quat_to_rot(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(r1, r2, atol=1e-6)


def test_compose_chain_matches_manual():
    # first link rotation is identity, so the second link's translation adds unrotated
    chain = [([0, 0, 0.033], [0, 0, 0]), ([0.1, 0, 0.06], [0, 0.5, 0])]
    T = compose(chain)
    assert np.allclose(T[:3, 3], [0.1, 0.0, 0.093], atol=1e-6)


if __name__ == "__main__":
    import sys
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except Exception:
            failed += 1
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
    sys.exit(1 if failed else 0)

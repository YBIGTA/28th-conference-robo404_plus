"""
Pure-numpy 3D->2D geometry for auto-labelling Gazebo frames.

Given a pinhole camera (intrinsics K), the camera pose in the world, and a
ground-truth object (world pose + half extents), produce a 2D bounding box and
the camera-space distance. No ROS / tf2 here so the math is unit-testable.

Camera frame follows the REP-103 optical convention: x right, y down, z forward.
"""
import math

import numpy as np


def rpy_to_rot(roll, pitch, yaw):
    """ZYX intrinsic (URDF rpy) -> 3x3 rotation."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_to_rot(x, y, z, w):
    """Quaternion -> 3x3 rotation."""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def make_tf(translation, rot):
    """3-vector + 3x3 rotation -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = translation
    return T


def invert_tf(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def compose(chain):
    """Compose a list of (xyz, rpy) links into one 4x4 transform."""
    T = np.eye(4)
    for xyz, rpy in chain:
        T = T @ make_tf(np.asarray(xyz, float), rpy_to_rot(*rpy))
    return T


def _aabb_corners(half):
    hx, hy, hz = half
    return np.array([[sx * hx, sy * hy, sz * hz]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def project_object(K, T_cam_world, center_world, quat_world, half_extents, img_w, img_h):
    """
    Returns a dict with bbox / distance, or None if the object is not in view.

      K            : 3x3 intrinsics
      T_cam_world  : 4x4 world->camera-optical transform
      center_world : (3,) object centre in world
      quat_world   : (4,) object orientation xyzw in world
      half_extents : (3,) object half sizes in its own frame
    """
    R = quat_to_rot(*quat_world)
    corners_world = (R @ _aabb_corners(half_extents).T).T + np.asarray(center_world, float)

    # to camera frame
    ones = np.ones((corners_world.shape[0], 1))
    cam = (T_cam_world @ np.hstack([corners_world, ones]).T).T[:, :3]
    center_cam = (T_cam_world @ np.array([*center_world, 1.0]))[:3]

    if center_cam[2] <= 0:                       # object centre behind camera
        return None

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    in_front = cam[:, 2] > 1e-3
    if in_front.sum() < 1:
        return None
    z = cam[in_front, 2]
    u = fx * cam[in_front, 0] / z + cx
    v = fy * cam[in_front, 1] / z + cy

    x1, y1 = float(u.min()), float(v.min())
    x2, y2 = float(u.max()), float(v.max())
    # clip to image
    cx1, cy1 = max(0.0, x1), max(0.0, y1)
    cx2, cy2 = min(float(img_w), x2), min(float(img_h), y2)
    if cx2 - cx1 < 1.0 or cy2 - cy1 < 1.0:       # fully / nearly out of frame
        return None

    bw, bh = cx2 - cx1, cy2 - cy1
    return {
        "bbox_xyxy_px": [round(cx1, 1), round(cy1, 1), round(cx2, 1), round(cy2, 1)],
        "bbox_xywh_norm": [
            round((cx1 + bw / 2) / img_w, 6), round((cy1 + bh / 2) / img_h, 6),
            round(bw / img_w, 6), round(bh / img_h, 6),
        ],
        "distance_m": round(float(np.linalg.norm(center_cam)), 3),
        "truncated": bool(x1 < 0 or y1 < 0 or x2 > img_w or y2 > img_h),
    }


def intrinsics_from_camera_info(k_row_major):
    """sensor_msgs/CameraInfo.k (length-9 row-major) -> 3x3."""
    return np.array(k_row_major, float).reshape(3, 3)

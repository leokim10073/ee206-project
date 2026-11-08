"""Camera pose and intrinsics from the values in sim.yaml.

Quaternions are (w, x, y, z). The ROS optical frame has z forward, x right, y down;
a MuJoCo camera looks along its -z axis with y up and x right.
"""

import math

import numpy as np


def _look_at_axes(position, look_at, up):
    forward = np.asarray(look_at, float) - np.asarray(position, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(up, float))
    right /= np.linalg.norm(right)
    image_up = np.cross(right, forward)
    return forward, right, image_up


def matrix_to_quat(R):
    """Rotation matrix to unit quaternion (w, x, y, z), w >= 0."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return tuple(float(v) for v in q / np.linalg.norm(q))


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def look_at_quaternion(position, look_at, up):
    """Orientation of the ROS optical frame in world."""
    forward, right, image_up = _look_at_axes(position, look_at, up)
    return matrix_to_quat(np.column_stack([right, -image_up, forward]))


def mujoco_camera_quaternion(position, look_at, up):
    """Orientation of the MJCF camera in world (looks along -z, y up)."""
    forward, right, image_up = _look_at_axes(position, look_at, up)
    return matrix_to_quat(np.column_stack([right, image_up, -forward]))


def pinhole_intrinsics(width, height, hfov_deg):
    """(fx, fy, cx, cy) for square pixels and the given horizontal field of view."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return fx, fx, (width - 1) / 2.0, (height - 1) / 2.0

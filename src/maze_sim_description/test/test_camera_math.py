import math

import numpy as np
import pytest

from maze_sim_description.camera_math import (
    look_at_quaternion,
    mujoco_camera_quaternion,
    pinhole_intrinsics,
    quat_to_matrix,
)


def test_intrinsics_c922():
    fx, fy, cx, cy = pinhole_intrinsics(1280, 720, 70.42)
    assert fx == pytest.approx(640 / math.tan(math.radians(35.21)), rel=1e-3)
    assert fx == fy and (cx, cy) == (639.5, 359.5)


def test_optical_frame_looks_down():
    q = look_at_quaternion((0.5, 0, 1.4), (0.5, 0, 0.4), (1, 0, 0))
    R = quat_to_matrix(q)
    assert np.allclose(R[:, 2], [0, 0, -1])  # optical z points at the board
    assert np.allclose(R[:, 1], [-1, 0, 0])  # image down = -up


def test_mujoco_and_optical_agree():
    args = ((0.3, 0.2, 1.2), (0.5, 0, 0.4), (1, 0, 0))
    Ro = quat_to_matrix(look_at_quaternion(*args))
    Rm = quat_to_matrix(mujoco_camera_quaternion(*args))
    assert np.allclose(-Rm[:, 2], Ro[:, 2])  # MuJoCo looks along -z, optical along +z
    assert np.allclose(Rm[:, 1], -Ro[:, 1])  # MuJoCo y up, optical y down
    assert np.allclose(Rm[:, 0], Ro[:, 0])  # both x right


def test_quaternions_are_unit():
    q = look_at_quaternion((0.3, 0.2, 1.2), (0.5, 0, 0.4), (1, 0, 0))
    assert np.linalg.norm(q) == pytest.approx(1.0)

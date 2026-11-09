"""Board pose, pixel <-> board geometry and rectification on synthetic camera images."""

import dataclasses

import cv2
import numpy as np
import pytest

from conftest import board_in_camera, marker_image, render_board
from maze_perception.board_pose import (
    BoardPose, Camera, board_to_rectified, estimate_board_pose, rectified_to_board, rectify,
)
from maze_perception.config import RectifiedSettings
from maze_perception.markers import MarkerDetector


def _angle_deg(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


@pytest.fixture(scope="module")
def detector(cfg):
    return MarkerDetector(cfg.board.dictionary, ids=cfg.board.markers)


@pytest.mark.parametrize("tilt, yaw, offset", [
    ((0, 0), 90, (0, 0)),
    ((5, 0), 90, (0.02, -0.01)),
    ((0, -5), 90, (-0.03, 0.02)),
    ((4, 4), 80, (0, 0)),
    ((-6, 3), 100, (0.05, 0.03)),
])
def test_pose_from_synthetic_image(cfg, camera, detector, tilt, yaw, offset):
    R, t = board_in_camera(height=1.0, tilt_deg=tilt, yaw_deg=yaw, offset=offset)
    img = render_board(cfg, camera, R, t)
    corners = detector.detect(img)
    assert sorted(corners) == [0, 1, 2, 3]
    pose = estimate_board_pose(corners, cfg.board, camera, cfg.pose)
    assert pose is not None
    assert pose.marker_ids == (0, 1, 2, 3)
    assert np.linalg.norm(pose.t - t) < 1e-3
    assert _angle_deg(pose.R, R) < 0.3
    assert pose.reprojection_px < 1.0


def test_detector_ignores_ids_not_on_the_board(cfg, camera):
    R, t = board_in_camera()
    img = render_board(cfg, camera, R, t)
    assert sorted(MarkerDetector(cfg.board.dictionary, ids=[0, 2]).detect(img)) == [0, 2]


def test_detector_drops_an_id_seen_twice(cfg, detector):
    img = np.full((400, 800), 128, np.uint8)
    for x0 in (50, 450):
        img[50:350, x0:x0 + 300] = 255
        img[100:300, x0 + 50:x0 + 250] = marker_image(cfg.board.dictionary, 0, 200)
    img[100:300, 300:400] = 255  # keep the two stickers apart
    assert detector.detect(img) == {}


def test_min_markers(cfg, camera, detector):
    R, t = board_in_camera(tilt_deg=(5, -3))
    corners = detector.detect(render_board(cfg, camera, R, t))
    two = {i: corners[i] for i in (1, 3)}
    assert estimate_board_pose(two, cfg.board, camera, cfg.pose) is None  # min_markers: 3
    truth = BoardPose(R=R, t=t, marker_ids=(), reprojection_px=0.0)
    grid = np.array([[x, y, 0.007] for x in np.linspace(-0.14, 0.14, 8) for y in np.linspace(-0.14, 0.14, 8)])
    px = truth.board_to_pixel(grid, camera)
    for hidden in range(4):  # e.g. one marker behind the gripper
        three = {i: c for i, c in corners.items() if i != hidden}
        pose = estimate_board_pose(three, cfg.board, camera, cfg.pose)
        assert pose is not None
        assert np.max(np.linalg.norm(pose.pixel_to_board(px, camera, z=0.007) - grid, axis=1)) < 1.5e-3


def test_rejects_a_misread_marker(cfg, camera, detector):
    R, t = board_in_camera()
    corners = detector.detect(render_board(cfg, camera, R, t))
    corners[2] = corners[2] + np.array([12.0, 0.0])  # e.g. a marker partly behind the gripper
    assert estimate_board_pose(corners, cfg.board, camera, cfg.pose) is None
    loose = dataclasses.replace(cfg.pose, max_reprojection_px=50.0)
    assert estimate_board_pose(corners, cfg.board, camera, loose) is not None


def _pose(tilt=(3, -2)):
    R, t = board_in_camera(tilt_deg=tilt, offset=(0.01, 0.02))
    return BoardPose(R=R, t=t, marker_ids=(0, 1, 2, 3), reprojection_px=0.0)


@pytest.mark.parametrize("z", [0.0, 0.007])
def test_pixel_board_round_trip(camera, z):
    pose = _pose()
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-0.15, 0.15, (50, 2)), np.full(50, z)])
    back = pose.pixel_to_board(pose.board_to_pixel(pts, camera), camera, z=z)
    np.testing.assert_allclose(back, pts, atol=1e-9)


def test_pixel_board_round_trip_with_distortion(camera):
    cam = dataclasses.replace(camera, D=np.array([0.08, -0.15, 0.001, -0.0005, 0.03]))
    pose = _pose()
    pts = np.array([[0.14, 0.14, 0.0], [-0.14, 0.1, 0.0], [0.0, -0.14, 0.0]])
    np.testing.assert_allclose(pose.pixel_to_board(pose.board_to_pixel(pts, cam), cam), pts, atol=1e-6)


def test_quaternion_matches_rotation():
    pose = _pose()
    x, y, z, w = pose.quaternion()
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    np.testing.assert_allclose(R, pose.R, atol=1e-9)


def test_plane_homography_matches_projection(camera):
    pose = _pose()
    xy = np.array([[0.1, -0.05], [-0.12, 0.13]])
    for z in (0.0, 0.018):
        h = (pose.plane_homography(camera, z) @ np.column_stack([xy, np.ones(2)]).T).T
        np.testing.assert_allclose(h[:, :2] / h[:, 2:], pose.board_to_pixel(np.column_stack([xy, [z, z]]), camera), atol=1e-9)


def test_rectified_pixel_mapping(cfg):
    s = cfg.rectified
    # pixel centers: the top-left pixel is half a pixel in from the plate's -x, +y corner
    np.testing.assert_allclose(rectified_to_board([[0, 0]], s), [[-0.15 + 0.00025, 0.15 - 0.00025]])
    np.testing.assert_allclose(rectified_to_board([[300, 300]], s), [[0.00025, -0.00025]], atol=1e-12)
    pts = np.array([[0.1, 0.02], [-0.07, -0.13]])
    np.testing.assert_allclose(rectified_to_board(board_to_rectified(pts, s), s), pts, atol=1e-12)


@pytest.mark.parametrize("distorted", [False, True])
def test_rectified_image_puts_markers_where_the_board_says(cfg, camera, detector, distorted):
    # a rectified view wide enough to include the corner markers: they land at their board position
    R, t = board_in_camera(tilt_deg=(4, -3), yaw_deg=95, offset=(0.02, -0.01))
    img = render_board(cfg, camera, R, t)
    pose = estimate_board_pose(detector.detect(img), cfg.board, camera, cfg.pose)
    # a tiny distortion takes the remap path but leaves the image as the warp path would see it
    cam = dataclasses.replace(camera, D=np.array([1e-12, 0, 0, 0, 0])) if distorted else camera
    wide = RectifiedSettings(x_range=(-0.16, 0.16), y_range=(-0.21, 0.21), px_per_m=2000.0)
    top = rectify(img, pose, cam, wide, z=cfg.board.markers[0][2])
    assert top.shape == (840, 640)
    found = detector.detect(top)
    assert sorted(found) == [0, 1, 2, 3]
    for mid, c in found.items():
        want = board_to_rectified(cfg.board.marker_corners(mid)[:, :2], wide)
        assert np.max(np.linalg.norm(c - want, axis=1)) < 2.0  # px, at 0.5 mm per px

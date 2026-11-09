"""Board pose from the ArUco corners, and the geometry built on it: pixel <-> board plane, and a
rectified top-down image of the board.

Frames: the camera frame is the ROS optical frame (z forward, x right, y down); the board frame
is the sim's (origin at the plate center, z up, z = 0 on the maze floor), in m.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

from maze_perception.config import BoardModel, PoseSettings, RectifiedSettings


@dataclass(frozen=True)
class Camera:
    """Pinhole intrinsics and distortion, as in sensor_msgs/CameraInfo (plumb_bob)."""

    K: np.ndarray
    D: np.ndarray
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, k: Sequence[float], d: Sequence[float], width: int, height: int) -> "Camera":
        return cls(np.asarray(k, float).reshape(3, 3), np.asarray(d, float).ravel(), int(width), int(height))

    @property
    def has_distortion(self) -> bool:
        return self.D.size > 0 and bool(np.any(self.D != 0.0))


@dataclass(frozen=True)
class BoardPose:
    """The board in the camera frame: p_camera = R @ p_board + t."""

    R: np.ndarray
    t: np.ndarray
    marker_ids: Tuple[int, ...]
    reprojection_px: float  # worst corner, px

    @property
    def rvec(self) -> np.ndarray:
        return cv2.Rodrigues(self.R)[0]

    def quaternion(self) -> Tuple[float, float, float, float]:
        """Board orientation in the camera frame as (x, y, z, w), the order ROS messages use."""
        R = self.R
        w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
        y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
        z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
        q = np.array([x, y, z, w])
        return tuple(float(v) for v in q / np.linalg.norm(q))

    def board_to_pixel(self, points: np.ndarray, camera: Camera) -> np.ndarray:
        """(N, 3) board points -> (N, 2) pixels, with the camera's distortion."""
        px, _ = cv2.projectPoints(np.asarray(points, float).reshape(-1, 1, 3), self.rvec, self.t, camera.K, camera.D)
        return px.reshape(-1, 2)

    def pixel_to_board(self, pixels: np.ndarray, camera: Camera, z: float = 0.0) -> np.ndarray:
        """(N, 2) pixels -> (N, 3) board points where each pixel's ray meets the plane at height z
        above the floor (z = ball radius for the ball's center). NaN for a ray parallel to it."""
        uv = np.asarray(pixels, float).reshape(-1, 1, 2)
        n = cv2.undistortPoints(uv, camera.K, camera.D).reshape(-1, 2)
        rays = np.column_stack([n, np.ones(len(n))]) @ self.R  # rows: R^T @ ray, in the board frame
        origin = -self.R.T @ self.t
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (z - origin[2]) / rays[:, 2]
        s = np.where(s > 0, s, np.nan)  # behind the camera
        return origin + s[:, None] * rays

    def plane_homography(self, camera: Camera, z: float = 0.0) -> np.ndarray:
        """3x3 H with pixel ~ H @ (x, y, 1) for board points (x, y, z); ignores distortion."""
        return camera.K @ np.column_stack([self.R[:, 0], self.R[:, 1], self.R[:, 2] * z + self.t])


def estimate_board_pose(corners: Dict[int, np.ndarray], model: BoardModel, camera: Camera,
                        settings: PoseSettings) -> Optional[BoardPose]:
    """Fit the board pose to the detected marker corners ({id: (4, 2) px}).

    None when fewer than settings.min_markers board markers were seen, when the fit puts the
    board behind the camera or facing away from it, or when a corner reprojects farther than
    settings.max_reprojection_px (a misread or partly hidden marker).
    """
    ids = tuple(sorted(i for i in corners if i in model.markers))
    if len(ids) < settings.min_markers:
        return None
    obj = np.vstack([model.marker_corners(i) for i in ids])
    img = np.vstack([np.asarray(corners[i], float).reshape(4, 2) for i in ids])
    # IPPE: the planar solver, no initial guess needed; then Levenberg-Marquardt on all corners
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera.K, camera.D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, camera.K, camera.D, rvec, tvec)
    R, t = cv2.Rodrigues(rvec)[0], tvec.ravel()
    if t[2] <= 0 or R[:, 2] @ t >= 0:  # board z must point back toward the camera
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera.K, camera.D)
    err = float(np.max(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)))
    if err > settings.max_reprojection_px:
        return None
    return BoardPose(R=R, t=t, marker_ids=ids, reprojection_px=err)


def _rectified_to_board_matrix(settings: RectifiedSettings) -> np.ndarray:
    """3x3 A with (x, y, 1) = A @ (col, row, 1); pixel centers, board +y up the image."""
    s = settings.px_per_m
    x0, y1 = settings.x_range[0], settings.y_range[1]
    return np.array([[1 / s, 0, x0 + 0.5 / s], [0, -1 / s, y1 - 0.5 / s], [0, 0, 1]])


def rectified_to_board(pixels: np.ndarray, settings: RectifiedSettings) -> np.ndarray:
    """(N, 2) rectified-image pixels (col, row) -> (N, 2) board (x, y), m."""
    p = np.asarray(pixels, float).reshape(-1, 2)
    return (np.column_stack([p, np.ones(len(p))]) @ _rectified_to_board_matrix(settings).T)[:, :2]


def board_to_rectified(points: np.ndarray, settings: RectifiedSettings) -> np.ndarray:
    """(N, 2) board (x, y), m -> (N, 2) rectified-image pixels (col, row)."""
    p = np.asarray(points, float).reshape(-1, 2)
    return (np.column_stack([p, np.ones(len(p))]) @ np.linalg.inv(_rectified_to_board_matrix(settings)).T)[:, :2]


def rectify(image: np.ndarray, pose: BoardPose, camera: Camera, settings: RectifiedSettings,
            z: float = 0.0) -> np.ndarray:
    """Top-down image of the board plane at height z: column = board +x, up = board +y,
    settings.px_per_m pixels per meter. Areas outside the camera image are black."""
    w, h = settings.size
    A = _rectified_to_board_matrix(settings)
    if not camera.has_distortion:
        M = pose.plane_homography(camera, z) @ A
        return cv2.warpPerspective(image, M, (w, h), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    cols, rows = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    xy = rectified_to_board(np.column_stack([cols.ravel(), rows.ravel()]), settings)
    px = pose.board_to_pixel(np.column_stack([xy, np.full(len(xy), z)]), camera).astype(np.float32)
    return cv2.remap(image, px[:, 0].reshape(h, w), px[:, 1].reshape(h, w), cv2.INTER_LINEAR)

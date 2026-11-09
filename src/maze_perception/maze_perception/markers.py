"""ArUco detection that works on OpenCV 4.5 (the maze-sim image) and on OpenCV >= 4.7."""

from typing import Dict, Iterable, Optional

import cv2
import numpy as np


def _new_api() -> bool:
    return hasattr(cv2.aruco, "ArucoDetector")


def opencv_version_warning() -> str:
    """A warning to log when this OpenCV is the old ArUco detector, else ''.

    OpenCV 4.5 (Ubuntu 22.04's python3-opencv) misses markers that 4.7+ finds: on synthetic
    images of the board 1 m below the camera, 4.5.5 found all four markers in 0-7 of 8 poses
    depending on the color around the stickers, 4.10 in 8 of 8.
    """
    if _new_api():
        return ""
    return (f"OpenCV {cv2.__version__} has the old ArUco detector, which misses markers; "
            "use opencv >= 4.7 (rebuild the maze-sim image)")


class MarkerDetector:
    """Finds the board's ArUco markers in an image.

    Corners are refined to sub-pixel accuracy; the pose is fitted to them, so this matters more
    than detection speed. An id seen twice in one image is dropped, since its corners are ambiguous.
    """

    def __init__(self, dictionary: str, ids: Optional[Iterable[int]] = None):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV has no aruco module: install opencv-contrib (or opencv >= 4.7)")
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f"unknown ArUco dictionary {dictionary!r}")
        self.ids = None if ids is None else frozenset(int(i) for i in ids)
        dict_id = getattr(cv2.aruco, dictionary)
        if _new_api():
            d = cv2.aruco.getPredefinedDictionary(dict_id)
            params = cv2.aruco.DetectorParameters()
        else:
            d = cv2.aruco.Dictionary_get(dict_id)
            params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._dictionary, self._params = d, params
        self._detector = cv2.aruco.ArucoDetector(d, params) if _new_api() else None

    def detect(self, image: np.ndarray) -> Dict[int, np.ndarray]:
        """{id: (4, 2) float corners in pixels}, OpenCV's corner order. image: gray or RGB."""
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dictionary, parameters=self._params)
        if ids is None:
            return {}
        ids = [int(i) for i in np.asarray(ids).ravel()]
        out = {}
        for i, c in zip(ids, corners):
            if ids.count(i) > 1 or (self.ids is not None and i not in self.ids):
                continue
            out[i] = np.asarray(c, dtype=np.float64).reshape(4, 2)
        return out

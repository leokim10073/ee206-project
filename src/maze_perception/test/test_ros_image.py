from types import SimpleNamespace

import numpy as np
import pytest

from maze_perception.ros_image import image_to_rgb


def _msg(img, encoding, pad=0):
    h, w = img.shape[:2]
    row = img.reshape(h, -1)
    rows = np.hstack([row, np.zeros((h, pad), np.uint8)])
    return SimpleNamespace(height=h, width=w, encoding=encoding, step=rows.shape[1], data=rows.tobytes())


@pytest.fixture
def rgb():
    return np.random.default_rng(0).integers(0, 256, (4, 5, 3), dtype=np.uint8)


@pytest.mark.parametrize("pad", [0, 3])
def test_encodings(rgb, pad):
    np.testing.assert_array_equal(image_to_rgb(_msg(rgb, "rgb8", pad)), rgb)
    np.testing.assert_array_equal(image_to_rgb(_msg(rgb[:, :, ::-1].copy(), "bgr8", pad)), rgb)
    alpha = np.full((4, 5, 1), 7, np.uint8)
    np.testing.assert_array_equal(image_to_rgb(_msg(np.concatenate([rgb, alpha], 2), "rgba8", pad)), rgb)
    np.testing.assert_array_equal(image_to_rgb(_msg(np.concatenate([rgb[:, :, ::-1], alpha], 2), "bgra8", pad)), rgb)
    np.testing.assert_array_equal(image_to_rgb(_msg(rgb[:, :, 0].copy(), "mono8", pad)), rgb[:, :, 0])


def test_unsupported_encoding(rgb):
    with pytest.raises(ValueError, match="yuv422"):
        image_to_rgb(_msg(rgb, "yuv422"))

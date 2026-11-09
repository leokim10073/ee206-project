"""sensor_msgs/Image <-> numpy, without cv_bridge."""

import numpy as np

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def image_to_rgb(msg) -> np.ndarray:
    """sensor_msgs/Image -> (h, w, 3) uint8 RGB, or (h, w) for mono8. Honors `step` (row padding)."""
    if msg.encoding not in _CHANNELS:
        raise ValueError(f"unsupported image encoding {msg.encoding!r} (want one of {sorted(_CHANNELS)})")
    n = _CHANNELS[msg.encoding]
    data = msg.data if not isinstance(msg.data, list) else bytes(msg.data)  # rclpy gives array('B')
    rows = np.frombuffer(data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * n].reshape(msg.height, msg.width, n) if n > 1 else rows[:, :msg.width]
    if msg.encoding in ("bgr8", "bgra8"):
        img = img[:, :, 2::-1]
    elif n == 4:
        img = img[:, :, :3]
    return np.ascontiguousarray(img)


def rgb_to_image(img: np.ndarray, header):
    """(h, w, 3) RGB or (h, w) gray uint8 -> sensor_msgs/Image (rgb8 / mono8)."""
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "rgb8" if img.ndim == 3 else "mono8"
    msg.step = msg.width * (3 if img.ndim == 3 else 1)
    msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    return msg

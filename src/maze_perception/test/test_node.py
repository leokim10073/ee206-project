"""board_tracker end to end over ROS: synthetic camera_info and image in, board pose out.
Needs ROS 2 (run inside maze-sim after colcon build); skipped without rclpy."""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from conftest import PKG, board_in_camera, render_board  # noqa: E402
from maze_perception.board_node import BoardTracker  # noqa: E402
from maze_perception.ros_image import rgb_to_image  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_board_pose_published(ros, cfg, camera):
    tracker = BoardTracker(parameter_overrides=[
        Parameter("config", value=str(PKG / "config" / "perception.yaml")),
        Parameter("image_topic", value="/test/image"),
        Parameter("camera_info_topic", value="/test/camera_info"),
    ])
    peer = rclpy.create_node("board_tracker_test")
    executor = SingleThreadedExecutor()
    executor.add_node(tracker)
    executor.add_node(peer)

    R, t = board_in_camera(tilt_deg=(3, -2), offset=(0.01, 0.0))
    gray = render_board(cfg, camera, R, t)
    header = Header(frame_id="camera_optical_frame")
    image = rgb_to_image(np.dstack([gray] * 3), header)
    info = CameraInfo(header=header, width=camera.width, height=camera.height, distortion_model="plumb_bob",
                      k=camera.K.ravel().tolist(), d=[0.0] * 5)

    poses, boards = [], []
    peer.create_subscription(PoseStamped, "/perception/board_pose", poses.append, 10)
    peer.create_subscription(Image, "/perception/board_image", boards.append, 10)
    info_pub = peer.create_publisher(CameraInfo, "/test/camera_info", 10)
    image_pub = peer.create_publisher(Image, "/test/image", 10)
    deadline = time.monotonic() + 15.0
    while (not poses or not boards) and time.monotonic() < deadline:
        info_pub.publish(info)
        image_pub.publish(image)
        executor.spin_once(timeout_sec=0.1)
    tracker.destroy_node()
    peer.destroy_node()

    assert poses, "no /perception/board_pose"
    p = poses[-1]
    assert p.header.frame_id == "camera_optical_frame"
    np.testing.assert_allclose([p.pose.position.x, p.pose.position.y, p.pose.position.z], t, atol=1e-3)
    assert boards and (boards[-1].width, boards[-1].height) == cfg.rectified.size

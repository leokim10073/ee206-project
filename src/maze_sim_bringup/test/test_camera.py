"""The overhead camera publishes like the C922 will, from the pose in sim.yaml."""

import struct
import unittest

import numpy as np
import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from maze_sim_bringup.sim_tools_node import RATE_WINDOW_S
from maze_sim_description.board_assets import load_board_assets
from maze_sim_description.camera_math import look_at_quaternion, pinhole_intrinsics
from maze_sim_description.config import resolve_path
from sim_helpers import default_config, pose_matrix, sim_launch_description, spin_until, wait_for_startup


def _raw_stamp(serialized):
    """header.stamp of a CDR-serialized message: 4-byte encapsulation, then int32 sec, uint32 nanosec."""
    sec, nsec = struct.unpack_from("<iI", serialized, 4)
    return sec + nsec * 1e-9


@pytest.mark.launch_test
def generate_test_description():
    return sim_launch_description(camera="true")


class TestCamera(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_camera_test")
        cls.cfg = default_config()
        wait_for_startup(cls.node)
        cls.images, cls.infos, cls.board = [], [], []
        # raw: read only the header stamp, so the test does not slow the publisher it measures
        cls.node.create_subscription(Image, "/camera/image_raw", lambda b: cls.images.append(_raw_stamp(b)), 200, raw=True)
        cls.node.create_subscription(CameraInfo, "/camera/camera_info", cls.infos.append, 100)
        cls.node.create_subscription(PoseStamped, "/sim/board/pose", cls.board.append, 10)
        cls.ball = []
        cls.node.create_subscription(PoseStamped, "/sim/ball/pose", cls.ball.append, 10)
        spin_until(cls.node, lambda: len(cls.images) > 1 and cls.images[-1] - cls.images[0] >= 3.0, 60.0)
        # measured before any test deserializes a full image in this executor and drops frames
        cls.image_window = list(cls.images[1:])
        cls.info_window = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in cls.infos[1:]]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def K(self):
        return np.array(self.infos[-1].k).reshape(3, 3)

    def one_image(self):
        got = []
        sub = self.node.create_subscription(Image, "/camera/image_raw", got.append, 1)
        spin_until(self.node, lambda: got, 10.0)
        self.node.destroy_subscription(sub)
        return got[-1]

    def test_camera_rate(self, proc_output):
        img = self.one_image()
        self.assertEqual((img.width, img.height, img.encoding), (*self.cfg.camera.resolution, "rgb8"))
        rate = lambda t: (len(t) - 1) / (t[-1] - t[0])
        # Published rate: the plugin sends camera_info with every image, same stamp, and the small
        # message is never dropped on the way here. A GPU too slow to render each frame within its
        # slot (e.g. Intel Iris Xe: ~55 Hz for 60) is a hardware limit, not a bug, but must not pass
        # silently: then the sim has to say so.
        published = rate(self.info_window)
        if published < 0.95 * self.cfg.camera.fps:
            proc_output.assertWaitFor("below camera.fps", timeout=3 * RATE_WINDOW_S)
        else:
            self.assertAlmostEqual(published, self.cfg.camera.fps, delta=0.05 * self.cfg.camera.fps)
        # Received rate: the image publisher keeps only the newest frame (depth 1), so a subscriber
        # that is slow to take a 2.7 MB frame loses some. A Python one gets ~93% on an idle machine
        # and ~85% while colcon runs other tests, so this only checks that images arrive steadily.
        self.assertGreater(rate(self.image_window), 0.5 * published)

    def test_camera_info_matches(self):
        spin_until(self.node, lambda: self.infos, 5.0)
        info = self.infos[-1]
        w, h = self.cfg.camera.resolution
        fx, fy, cx, cy = pinhole_intrinsics(w, h, self.cfg.camera.hfov_deg)
        K = self.K()
        # the plugin puts the principal point at (w/2, h/2), half a pixel from the ROS
        # pixel-centre convention (w-1)/2 used by pinhole_intrinsics
        np.testing.assert_allclose([K[0, 0], K[1, 1]], [fx, fy], rtol=1e-5)
        np.testing.assert_allclose([K[0, 2], K[1, 2]], [cx, cy], atol=0.5 + 1e-9)
        self.assertEqual((info.width, info.height), (w, h))
        self.assertTrue(all(d == 0 for d in info.d))
        self.assertEqual(info.header.frame_id, "camera_optical_frame")
        self.assertEqual(self.one_image().header.frame_id, "camera_optical_frame")

    def _camera_tf(self):
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self.node)
        spin_until(self.node, lambda: buf.can_transform("world", "camera_optical_frame", rclpy.time.Time()), 10.0)
        return buf.lookup_transform("world", "camera_optical_frame", rclpy.time.Time()).transform

    def test_camera_tf_matches_config(self):
        tf = self._camera_tf()
        c = self.cfg.camera
        np.testing.assert_allclose([tf.translation.x, tf.translation.y, tf.translation.z], c.position, atol=1e-6)
        want = np.array(look_at_quaternion(c.position, c.look_at, c.up))  # (w, x, y, z)
        got = np.array([tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
        self.assertGreater(abs(np.dot(want, got)), 1 - 1e-6)

    def _project(self, p_world):
        tf = self._camera_tf()
        from geometry_msgs.msg import Pose

        T = pose_matrix(Pose(position=type(Pose().position)(x=tf.translation.x, y=tf.translation.y, z=tf.translation.z),
                             orientation=tf.rotation))
        p_cam = np.linalg.inv(T) @ np.append(p_world, 1.0)
        uvw = self.K() @ p_cam[:3]
        return uvw[:2] / uvw[2], p_cam[2]

    def test_board_in_frame(self):
        spin_until(self.node, lambda: self.board and self.infos, 5.0)
        assets = load_board_assets(resolve_path(self.cfg.board.mesh_dir, "."))
        T = pose_matrix(self.board[-1].pose)
        hx, hy = assets.extent_m[0] / 2, assets.extent_m[1] / 2
        w, h = self.cfg.camera.resolution
        for x, y in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)):
            (u, v), depth = self._project((T @ [x, y, 0, 1])[:3])
            self.assertGreater(depth, 0)
            self.assertTrue(0 <= u < w and 0 <= v < h, (x, y, u, v))

    def test_ball_visible_where_projected(self):
        # checks image orientation, intrinsics and camera TF together: the grey ball shows up
        # in the image at the pixel its ground-truth pose projects to
        spin_until(self.node, lambda: self.ball and self.infos, 5.0)
        p = self.ball[-1].pose.position
        (u, v), _ = self._project([p.x, p.y, p.z])
        img = self.one_image()
        rgb = np.frombuffer(bytes(img.data), np.uint8).reshape(img.height, img.width, 3).astype(float)
        u, v = int(round(u)), int(round(v))
        self.assertTrue(2 <= u < img.width - 40 and 2 <= v < img.height - 2, (u, v, p, self.board[-1].pose.position))
        patch = rgb[v - 2 : v + 3, u - 2 : u + 3].reshape(-1, 3).mean(axis=0)
        board = rgb[v - 2 : v + 3, u + 30 : u + 35].reshape(-1, 3).mean(axis=0)
        # board wood is (0.82, 0.72, 0.55); the ball is bluish grey (0.75, 0.75, 0.8)
        self.assertGreater(patch[2] / max(patch[0], 1), board[2] / max(board[0], 1) + 0.15, (patch, board))

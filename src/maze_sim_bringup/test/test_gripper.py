"""/toggle_gripper opens and closes the 2F-85 as in lab 5; opening drops the board, /sim/reset regrasps it."""

import unittest

import numpy as np
import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from sim_helpers import UR_JOINTS, pose_matrix, sim_launch_description, spin_until, stamp, wait_for_startup


@pytest.mark.launch_test
def generate_test_description():
    return sim_launch_description()


class TestGripper(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_gripper_test")
        wait_for_startup(cls.node)
        cls.board, cls.done, cls.joints = [], [], []
        cls.node.create_subscription(PoseStamped, "/sim/board/pose", cls.board.append, 100)
        cls.node.create_subscription(Empty, "/sim/reset_done", cls.done.append, 10)  # volatile: new resets only
        cls.node.create_subscription(JointState, "/joint_states", cls.joints.append, 10)
        cls.tf = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf, cls.node)
        spin_until(cls.node, lambda: cls.board, 10.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # helpers -------------------------------------------------------------------------------

    def call(self, name):
        client = self.node.create_client(Trigger, name)
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), name)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), f"{name} did not answer")
        self.assertTrue(future.result().success, future.result().message)
        return future.result().message

    def toggle(self):
        return self.call("/toggle_gripper")

    def reset(self):
        n = len(self.done)
        self.call("/sim/reset")
        self.assertTrue(spin_until(self.node, lambda: len(self.done) > n, 5.0), "no /sim/reset_done")
        self.wait_sim(0.1)

    def wait_sim(self, seconds):
        t0 = stamp(self.board[-1])
        spin_until(self.node, lambda: stamp(self.board[-1]) - t0 >= seconds, 10.0 * seconds + 10.0)

    def board_in_tool0(self):
        spin_until(self.node, lambda: self.tf.can_transform("world", "tool0", rclpy.time.Time()), 10.0)
        t = self.tf.lookup_transform("world", "tool0", rclpy.time.Time()).transform
        tool0 = PoseStamped().pose
        tool0.position.x, tool0.position.y, tool0.position.z = t.translation.x, t.translation.y, t.translation.z
        tool0.orientation = t.rotation
        return np.linalg.inv(pose_matrix(tool0)) @ pose_matrix(self.board[-1].pose)

    # tests (unittest runs them in name order; they share one running sim) -------------------

    def test_1_toggle_reports_state(self):
        self.assertEqual(self.toggle(), "opened")
        self.assertEqual(self.toggle(), "closed")
        self.reset()

    def test_2_open_drops_board(self):
        z0, t0 = self.board[-1].pose.position.z, stamp(self.board[-1])
        self.assertEqual(self.toggle(), "opened")
        spin_until(self.node, lambda: z0 - self.board[-1].pose.position.z > 0.1 or stamp(self.board[-1]) - t0 > 1.0, 20.0)
        self.assertGreater(z0 - self.board[-1].pose.position.z, 0.1, "board did not fall after opening")
        self.assertLessEqual(stamp(self.board[-1]) - t0, 1.0 + 0.05)

    def test_3_reset_after_release_regrasps(self):
        self.reset()
        held = self.board_in_tool0()
        self.assertEqual(self.toggle(), "opened")
        self.wait_sim(1.0)
        self.reset()
        start = self.board_in_tool0()
        np.testing.assert_allclose(start[:3, 3], held[:3, 3], atol=1e-3)  # back in the grasp
        self.wait_sim(5.0)
        end = self.board_in_tool0()
        self.assertLess(np.linalg.norm(end[:3, 3] - start[:3, 3]), 1e-3, "board slipped after reset")
        # reset closed the gripper, so the next toggle opens it
        self.assertEqual(self.toggle(), "opened")
        self.assertEqual(self.toggle(), "closed")

    def test_4_joint_states_still_ur_only(self):
        spin_until(self.node, lambda: self.joints, 5.0)
        self.assertEqual(sorted(self.joints[-1].name), sorted(UR_JOINTS))

"""The sim exposes the UR7e interfaces the lab code uses, with the real robot's names and types."""

import os
import unittest

import numpy as np
import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from sim_helpers import UR_JOINTS, sim_launch_description, spin_until, wait_for_startup

EXPECTED_TOPICS = {
    "/joint_states": "sensor_msgs/msg/JointState",
    "/forward_velocity_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/scaled_joint_trajectory_controller/joint_trajectory": "trajectory_msgs/msg/JointTrajectory",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}


@pytest.mark.launch_test
def generate_test_description():
    return sim_launch_description()


class TestInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_interface_test")
        cls.controllers = wait_for_startup(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_controller_states(self):
        s = self.controllers
        self.assertEqual(s.get("scaled_joint_trajectory_controller"), "active", s)
        self.assertEqual(s.get("joint_state_broadcaster"), "active", s)
        self.assertEqual(s.get("gripper_controller"), "active", s)
        self.assertEqual(s.get("forward_velocity_controller"), "inactive", s)
        self.assertEqual(s.get("forward_position_controller"), "inactive", s)

    def test_topics_and_types(self):
        def found():
            have = dict(self.node.get_topic_names_and_types())
            return all(t in have and want in have[t] for t, want in EXPECTED_TOPICS.items())

        spin_until(self.node, found, 10.0)
        have = dict(self.node.get_topic_names_and_types())
        for topic, want in EXPECTED_TOPICS.items():
            self.assertIn(topic, have)
            self.assertIn(want, have[topic], topic)

    def test_trajectory_action_exists(self):
        client = ActionClient(
            self.node, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        client.destroy()

    def test_joint_states_are_ur_only(self):
        stamps, names = [], []

        def cb(msg):
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            names.append(list(msg.name))

        sub = self.node.create_subscription(JointState, "/joint_states", cb, 1000)
        spin_until(self.node, lambda: stamps and stamps[-1] - stamps[0] >= 2.0, 30.0)
        self.node.destroy_subscription(sub)
        self.assertTrue(stamps, "no /joint_states")
        self.assertTrue(all(sorted(n) == sorted(UR_JOINTS) for n in names), names[-1])
        span = stamps[-1] - stamps[0]
        rate = (len(stamps) - 1) / span
        self.assertAlmostEqual(rate, 500.0, delta=25.0)

    def test_tf_chain_matches_mujoco(self):
        # TF from ur_description and the MuJoCo tool0 site agree, so world means the same thing in both
        import mujoco
        import tf2_ros

        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self.node)
        spin_until(self.node, lambda: buf.can_transform("world", "tool0", rclpy.time.Time()), 10.0)
        self.assertTrue(buf.can_transform("base_link", "tool0", rclpy.time.Time()))
        js = []
        sub = self.node.create_subscription(JointState, "/joint_states", js.append, 10)
        spin_until(self.node, lambda: js, 5.0)
        self.node.destroy_subscription(sub)
        tf = buf.lookup_transform("world", "tool0", rclpy.time.Time()).transform

        ros_home = os.environ.get("ROS_HOME", os.path.expanduser("~/.ros"))
        model = mujoco.MjModel.from_xml_path(os.path.join(ros_home, "maze_sim", "scene.xml"))
        data = mujoco.MjData(model)
        for name, q in zip(js[-1].name, js[-1].position):
            data.joint(name).qpos = q
        mujoco.mj_kinematics(model, data)
        p = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        np.testing.assert_allclose(p, data.site("tool0").xpos, atol=1e-3)
        q_ros = np.array([tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
        q_mj = np.zeros(4)
        mujoco.mju_mat2Quat(q_mj, data.site("tool0").xmat)
        self.assertGreater(abs(np.dot(q_ros, q_mj)), np.cos(np.radians(0.5) / 2))

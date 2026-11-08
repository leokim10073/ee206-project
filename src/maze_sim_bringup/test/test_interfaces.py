"""The sim exposes the UR7e interfaces the lab code uses, with the real robot's names and types."""

import os
import time
import unittest

import launch
import launch_testing
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
EXPECTED_TOPICS = {
    "/joint_states": "sensor_msgs/msg/JointState",
    "/forward_velocity_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/scaled_joint_trajectory_controller/joint_trajectory": "trajectory_msgs/msg/JointTrajectory",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}
STARTUP_S = 60.0


@pytest.mark.launch_test
def generate_test_description():
    sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("maze_sim_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"viewer": "false", "camera": "false"}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def _spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        value = predicate()
        if value:
            return value
    return predicate()


def _controller_states(node):
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=1.0):
        return {}
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    node.destroy_client(client)
    if future.result() is None:
        return {}
    return {c.name: c.state for c in future.result().controller}


class TestInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_interface_test")
        want = {
            "scaled_joint_trajectory_controller": "active",
            "joint_state_broadcaster": "active",
            "gripper_controller": "active",
            "forward_velocity_controller": "inactive",
            "forward_position_controller": "inactive",
        }
        cls.controllers = _spin_until(
            cls.node,
            lambda: (lambda s: s if all(s.get(k) == v for k, v in want.items()) else None)(
                _controller_states(cls.node)
            ),
            STARTUP_S,
        ) or _controller_states(cls.node)

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

        _spin_until(self.node, found, 10.0)
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
        _spin_until(self.node, lambda: stamps and stamps[-1] - stamps[0] >= 2.0, 30.0)
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
        _spin_until(self.node, lambda: buf.can_transform("world", "tool0", rclpy.time.Time()), 10.0)
        self.assertTrue(buf.can_transform("base_link", "tool0", rclpy.time.Time()))
        js = []
        sub = self.node.create_subscription(JointState, "/joint_states", js.append, 10)
        _spin_until(self.node, lambda: js, 5.0)
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

"""The arm answers the lab's controllers the way the real UR7e does."""

import os
import subprocess
import sys
import time
import unittest

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from sim_helpers import UR_JOINTS, controller_states, sim_launch_description, spin_until, switch, wait_for_startup

TRAJ = "scaled_joint_trajectory_controller"
VEL = "forward_velocity_controller"
POS = "forward_position_controller"


@pytest.mark.launch_test
def generate_test_description():
    return sim_launch_description()


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class TestArmControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_arm_test")
        cls.states = []
        cls.node.create_subscription(JointState, "/joint_states", cls.states.append, 1000)
        cls.vel_pub = cls.node.create_publisher(Float64MultiArray, f"/{VEL}/commands", 10)
        cls.pos_pub = cls.node.create_publisher(Float64MultiArray, f"/{POS}/commands", 10)
        wait_for_startup(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # helpers -------------------------------------------------------------------------------

    def use(self, controller):
        """Make `controller` the one arm controller that is active."""
        active = {k for k, v in controller_states(self.node).items() if v == "active"} & {TRAJ, VEL, POS}
        if active != {controller}:
            self.assertTrue(switch(self.node, [controller], sorted(active - {controller})), controller)

    def q(self):
        spin_until(self.node, lambda: self.states, 5.0)
        m = self.states[-1]
        return np.array([m.position[m.name.index(j)] for j in UR_JOINTS])

    def wait_sim(self, seconds):
        """Spin until `seconds` of simulated time have passed (by /joint_states stamps)."""
        spin_until(self.node, lambda: self.states, 5.0)
        t0 = _stamp(self.states[-1])
        spin_until(self.node, lambda: _stamp(self.states[-1]) - t0 >= seconds, 10.0 * seconds + 10.0)

    def hold(self):
        self.vel_pub.publish(Float64MultiArray(data=[0.0] * 6))
        self.wait_sim(0.5)

    # tests (unittest runs them in name order; each one sets up the controller it needs) ------

    def test_1_velocity_tracks(self):
        self.use(VEL)
        self.hold()
        cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        self.vel_pub.publish(Float64MultiArray(data=cmd))
        self.wait_sim(0.2)
        start = len(self.states)
        self.wait_sim(0.5)
        v = [m.velocity[m.name.index("wrist_3_joint")] for m in self.states[start:]]
        self.assertAlmostEqual(np.mean(v), 0.1, delta=0.002)
        self.assertLess(np.max(np.abs(np.array(v) - 0.1)), 0.002 * 5)
        self.hold()

    def test_2_wrong_length_command_ignored(self):
        self.use(VEL)
        self.hold()
        before = self.q()
        clocks = []
        sub = self.node.create_subscription(Clock, "/clock", clocks.append, 10)
        self.vel_pub.publish(Float64MultiArray(data=[0.1] * 5))
        self.wait_sim(1.0)
        self.node.destroy_subscription(sub)
        self.assertLess(np.max(np.abs(self.q() - before)), 1e-3, (self.q() - before).round(4))
        self.assertTrue(clocks, "sim stopped publishing /clock")

    def test_3_position_reached(self):
        self.use(POS)
        target = self.q()
        target[UR_JOINTS.index("elbow_joint")] += 0.2
        self.pos_pub.publish(Float64MultiArray(data=list(target)))
        self.wait_sim(3.0)
        self.assertLess(np.max(np.abs(self.q() - target)), 0.01)
        target[UR_JOINTS.index("elbow_joint")] -= 0.2
        self.pos_pub.publish(Float64MultiArray(data=list(target)))
        self.wait_sim(3.0)

    def test_4_trajectory_goal(self):
        self.use(TRAJ)
        q0 = self.q()
        client = ActionClient(self.node, FollowJointTrajectory, f"/{TRAJ}/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR_JOINTS
        for t, dq in ((2, 0.1), (4, -0.1), (6, 0.0)):
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=list(q0 + dq), time_from_start=Duration(sec=t))
            )
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        self.assertTrue(sent.result().accepted)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=60.0)
        self.assertEqual(result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        client.destroy()

    def test_5_switch_both_ways(self):
        self.use(TRAJ)
        self.assertTrue(switch(self.node, [VEL], [TRAJ]))
        self.assertEqual(controller_states(self.node)[VEL], "active")
        self.hold()
        self.assertTrue(switch(self.node, [TRAJ], [VEL]))
        s = controller_states(self.node)
        self.assertEqual((s[TRAJ], s[VEL]), ("active", "inactive"))

    def test_6_lab7_style_node_unchanged(self):
        self.use(TRAJ)
        self.wait_sim(0.5)
        before = self.q()[UR_JOINTS.index("wrist_3_joint")]
        node = os.path.join(os.path.dirname(__file__), "lab7_style_node.py")
        sim0, wall0 = _stamp(self.states[-1]), time.monotonic()
        subprocess.run([sys.executable, node], check=True, timeout=120)  # wall clock, as lab code runs
        spin_until(self.node, lambda: False, 0.2)
        rtf = (_stamp(self.states[-1]) - sim0) / (time.monotonic() - wall0)
        self.wait_sim(0.5)
        moved = self.q()[UR_JOINTS.index("wrist_3_joint")] - before
        print(f"LAB7 moved={moved:.4f} rtf={rtf:.2f}", file=sys.stderr)
        self.assertAlmostEqual(moved, 0.2, delta=0.02, msg=f"real-time factor during the run: {rtf:.2f}")

"""Sim-only helpers under /sim/: ground-truth poses and a repeatable reset."""

import unittest

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from sim_helpers import UR_JOINTS, default_config, pose_matrix, sim_launch_description, spin_until, stamp, wait_for_startup


@pytest.mark.launch_test
def generate_test_description():
    return sim_launch_description()


class TestSimTools(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("maze_sim_tools_test")
        cls.cfg = default_config()
        wait_for_startup(cls.node)
        cls.ball, cls.board, cls.done = [], [], []
        cls.node.create_subscription(PoseStamped, "/sim/ball/pose", cls.ball.append, 100)
        cls.node.create_subscription(PoseStamped, "/sim/board/pose", cls.board.append, 100)
        cls.node.create_subscription(Empty, "/sim/reset_done", cls.done.append, 10)  # volatile: new resets only
        cls.joints = []
        cls.node.create_subscription(JointState, "/joint_states", cls.joints.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def reset(self):
        client = self.node.create_client(Trigger, "/sim/reset")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        n = len(self.done)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertTrue(future.result().success, future.result().message)
        self.assertTrue(spin_until(self.node, lambda: len(self.done) > n, 5.0), "no /sim/reset_done")
        self.node.destroy_client(client)
        t = stamp(self.ball[-1]) if self.ball else 0.0
        spin_until(self.node, lambda: self.ball and stamp(self.ball[-1]) > t + 0.1, 5.0)

    def test_poses_in_world(self):
        spin_until(self.node, lambda: self.ball and self.board, 10.0)
        self.assertEqual(self.ball[-1].header.frame_id, "world")
        self.assertEqual(self.board[-1].header.frame_id, "world")

    def test_ball_pose_matches_start(self):
        self.reset()
        T = pose_matrix(self.board[-1].pose)
        want = (T @ [*self.cfg.ball.start, self.cfg.ball.radius, 1.0])[:3]
        p = self.ball[-1].pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], want, atol=1e-4)
        q = self.joints[-1]
        got = [q.position[q.name.index(j)] for j in UR_JOINTS]
        np.testing.assert_allclose(got, self.cfg.robot.initial_joints, atol=2e-3)

    def _start_state(self):
        b, ball, q = self.board[-1].pose, self.ball[-1].pose.position, self.joints[-1]
        return np.array([
            b.position.x, b.position.y, b.position.z, b.orientation.x, b.orientation.y, b.orientation.z,
            ball.x, ball.y, *(q.position[q.name.index(j)] for j in UR_JOINTS),
        ])

    def _tilt_run(self):
        self.reset()
        start = self._start_state()
        client = ActionClient(self.node, FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        q0 = np.array(self.cfg.robot.initial_joints)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR_JOINTS
        for t, d in ((2, 0.05), (4, -0.05), (6, 0.0)):
            q = q0.copy()
            q[UR_JOINTS.index("wrist_1_joint")] += d
            goal.trajectory.points.append(JointTrajectoryPoint(positions=list(q), time_from_start=Duration(sec=t)))
        # start at a known sim time so both runs line up regardless of how long the goal takes to arrive
        t0 = stamp(self.ball[-1]) + 0.5
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = int(t0), int((t0 % 1) * 1e9)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=60.0)
        client.destroy()
        # ball xy at exact sim times every 0.1 s, interpolated from the 100 Hz stream, so a
        # dropped message does not shift a sample by 10 ms
        spin_until(self.node, lambda: stamp(self.ball[-1]) - t0 > 6.05, 10.0)
        t = np.array([stamp(m) - t0 for m in self.ball])
        xy = np.array([(m.pose.position.x, m.pose.position.y) for m in self.ball])
        samples = np.arange(61) * 0.1
        self.assertTrue(t[0] <= 0.0 and t[-1] >= samples[-1], "ball pose stream does not cover the run")
        return start, np.column_stack([np.interp(samples, t, xy[:, 0]), np.interp(samples, t, xy[:, 1])])

    def test_reset_is_repeatable(self):
        (sa, a), (sb, b) = self._tilt_run(), self._tilt_run()
        # what reset controls: board, ball and joints come back to the same state (0.1 mm, 0.1 mrad)
        np.testing.assert_allclose(sa, sb, atol=1e-4)
        # The plugin steps physics on its own thread and applies controller output as it arrives,
        # so commands land a step earlier or later from run to run; once the ball hits a wall
        # that jitter grows to ~1 cm. Compare the path while the ball rolls free (first 1.5 s;
        # measured up to 1.8 mm apart over five runs).
        moved = np.hypot(*(a - a[0]).T).max()
        self.assertGreater(moved, 0.005, "ball did not roll; the tilt is too small to test anything")
        diff = np.hypot(*(a - b).T)
        self.assertLess(diff[:16].max(), 3e-3)

    def test_ball_at_rest_after_reset(self):
        self.reset()
        p0, t0 = self.ball[-1].pose.position, stamp(self.ball[-1])
        spin_until(self.node, lambda: stamp(self.ball[-1]) - t0 >= 5.0, 30.0)
        p = self.ball[-1].pose.position
        self.assertLess(np.hypot(p.x - p0.x, p.y - p0.y), 2e-3, "ball rolled after reset: board not level")

    def test_rtf_logged(self, proc_output):
        proc_output.assertWaitFor("real-time factor:", timeout=15)

    def test_sim_time_has_no_gaps(self):
        clocks = []
        sub = self.node.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock.sec + m.clock.nanosec * 1e-9), 1000)
        spin_until(self.node, lambda: len(clocks) > 1 and clocks[-1] - clocks[0] >= 3.0, 30.0)
        self.node.destroy_subscription(sub)
        dt = np.diff(clocks)
        steps = dt / self.cfg.physics.timestep
        # /clock is dropped when the publisher is busy, so deltas are whole steps, not always one
        np.testing.assert_allclose(steps, np.round(steps), atol=1e-3)
        self.assertGreater(dt.min(), 0.0)
        self.assertLess(dt.max(), 0.1, "sim time jumped: physics steps were skipped")

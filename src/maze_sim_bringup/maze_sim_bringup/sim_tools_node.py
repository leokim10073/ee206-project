"""Sim-only helpers the MuJoCo plugin does not provide itself (the ball and board poses are
republished by the C++ pose_relay in maze_sim_nodes):

- the static TF world -> camera_optical_frame from sim.yaml
- /sim/reset (Trigger): arm controllers off, reset_world to keyframe "start", gripper closed,
  controllers back on with their reference at initial_joints, ball placed at rest on the board,
  then /sim/reset_done (latched)
- one reset as soon as the spawners are done: until the gripper controller gets a command the
  fingers hold position without squeezing and the arm sags with its controllers inactive, so the
  start state is only clean after this first reset
- a warning when the camera publishes below camera.fps
- every 10 s, "real-time factor: <x.xx>" (sim time elapsed / wall time elapsed)
"""

import sys
import threading
import time

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped, TransformStamped
from mujoco_ros2_control_msgs.msg import FreeJointState
from mujoco_ros2_control_msgs.srv import ResetWorld, SetFreeJointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty, Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from maze_sim_description.camera_math import look_at_quaternion
from maze_sim_description.config import load_sim_config

ARM_CONTROLLERS = ("scaled_joint_trajectory_controller", "forward_velocity_controller", "forward_position_controller")
GRIPPER_CLOSED = 0.8  # ctrlrange max of the finger actuator: close with the force limit
STARTUP_STATES = {
    "joint_state_broadcaster": "active",
    "scaled_joint_trajectory_controller": "active",
    "gripper_controller": "active",
    "forward_velocity_controller": "inactive",
    "forward_position_controller": "inactive",
}
UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
CALL_TIMEOUT_S = 10.0
GRIPPER_LEAD_S = 0.05  # sim time for the close command to reach the finger actuator
SETTLE_S = 1.0  # sim time for the arm to return to initial_joints after a reset (position PID settles in ~1 s)
BOARD_POSE_TOPIC = "/sim/board/pose"
RATE_WINDOW_S = 5.0
RTF_PERIOD_S = 10.0


class SimTools(Node):
    def __init__(self):
        super().__init__("sim_tools")
        self.cfg = load_sim_config(self.declare_parameter("config", "").value)
        self.camera = self.declare_parameter("camera", True).value
        group = self.group = ReentrantCallbackGroup()

        c = self.cfg.camera
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "world", "camera_optical_frame"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = c.position
        w, x, y, z = look_at_quaternion(c.position, c.look_at, c.up)
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = w, x, y, z
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers", callback_group=group)
        self.switch_cli = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=group)
        self.reset_cli = self.create_client(ResetWorld, "/mujoco_ros2_control_node/reset_world", callback_group=group)
        self.ball_cli = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state", callback_group=group
        )
        self.traj_pub = self.create_publisher(JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10)
        self.pos_pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reset_done = self.create_publisher(Empty, "/sim/reset_done", latched)
        self.create_service(Trigger, "/sim/reset", self.on_reset, callback_group=group)
        self.reset_lock = threading.Lock()
        # steady clock: a sim-time timer fires in a burst when /clock jumps at start-up
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.startup_timer = self.create_timer(0.5, self.startup_reset, callback_group=group, clock=wall)
        self.rtf_mark = None
        self.create_timer(RTF_PERIOD_S, self.log_rtf, clock=wall)

        if self.camera:
            self.frames = []
            # camera_info goes out with every frame and carries its stamp: count those instead of
            # taking a second copy of the 2.7 MB image stream
            self.create_subscription(CameraInfo, "/camera/camera_info", self.on_frame, 10)
            self.create_timer(RATE_WINDOW_S, self.check_camera_rate, clock=wall)

    # camera rate ---------------------------------------------------------------------------

    def on_frame(self, info):
        self.frames.append(info.header.stamp.sec + info.header.stamp.nanosec * 1e-9)

    def check_camera_rate(self):
        if not self.frames:
            return
        self.frames = [t for t in self.frames if t > self.frames[-1] - RATE_WINDOW_S]
        if len(self.frames) < 2:
            return
        rate = (len(self.frames) - 1) / (self.frames[-1] - self.frames[0])
        if rate < 0.95 * self.cfg.camera.fps:
            self.get_logger().warn(f"camera at {rate:.1f} fps (sim time), below camera.fps {self.cfg.camera.fps}")

    # real-time factor ----------------------------------------------------------------------

    def log_rtf(self):
        mark = (self.get_clock().now().nanoseconds * 1e-9, time.monotonic())
        last, self.rtf_mark = self.rtf_mark, mark
        if last is None or mark[0] <= last[0]:  # first tick, or sim time not running yet
            return
        level, text = rtf_message((mark[0] - last[0]) / (mark[1] - last[1]), self.cfg.physics.real_time_factor)
        getattr(self.get_logger(), level)(text)

    # reset ---------------------------------------------------------------------------------

    def startup_reset(self):
        if not self.list_cli.service_is_ready() or self.reset_lock.locked():
            return
        try:
            states = {c.name: c.state for c in self._call(self.list_cli, ListControllers.Request()).controller}
        except RuntimeError:
            return
        if any(states.get(name) != state for name, state in STARTUP_STATES.items()):
            return
        result = self.on_reset(None, Trigger.Response())
        if result.success:
            self.startup_timer.cancel()
            self.get_logger().info("start-up reset done")
        else:
            self.get_logger().warn(f"start-up reset failed, retrying: {result.message}")

    def on_reset(self, _request, response):
        with self.reset_lock:
            return self._reset(response)

    def _reset(self, response):
        released = []  # arm controllers switched off and not yet back on
        try:
            states = self._call(self.list_cli, ListControllers.Request()).controller
            active = [c.name for c in states if c.name in ARM_CONTROLLERS and c.state == "active"]
            # active controllers would pull the arm back to their old targets after the reset
            self._switch([], active)
            released = active
            # Close first: if the gripper was open, a command sent after reset_world arrives a few
            # steps late and the restored board slips out of the restored grasp in the meantime.
            self.gripper_pub.publish(Float64MultiArray(data=[GRIPPER_CLOSED]))
            self._sleep_sim(GRIPPER_LEAD_S)
            result = self._call(self.reset_cli, ResetWorld.Request(keyframe="start"))
            if not result.success:
                raise RuntimeError(f"reset_world failed: {result.message}")
            self._switch(active, [])
            released = []
            # A controller holds wherever the arm is when it activates, and the arm sags a little
            # under the board while no controller runs: send its reference back to initial_joints,
            # then put the ball back at rest on the level board.
            self._command_initial_joints(active)
            self._sleep_sim(SETTLE_S)
            self._place_ball()
        except Exception as e:  # report through the service instead of killing the node
            response.success, response.message = False, str(e)
            if released:  # a failed reset must not leave the arm sagging with no controller
                try:
                    self._switch(released, [])
                except Exception as e2:
                    response.message += f"; reactivating {released} also failed: {e2}"
            return response
        self.reset_done.publish(Empty())
        response.success, response.message = True, "reset to keyframe 'start'"
        return response

    def _command_initial_joints(self, active):
        q = list(self.cfg.robot.initial_joints)
        if "scaled_joint_trajectory_controller" in active:
            traj = JointTrajectory(joint_names=list(UR_JOINTS))
            traj.points = [JointTrajectoryPoint(positions=q, time_from_start=Duration(nanosec=int(SETTLE_S / 2 * 1e9)))]
            self.traj_pub.publish(traj)
        if "forward_position_controller" in active:
            self.pos_pub.publish(Float64MultiArray(data=q))
        if "forward_velocity_controller" in active:
            self.vel_pub.publish(Float64MultiArray(data=[0.0] * 6))

    def _sleep_sim(self, seconds):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < seconds * 1e9:
            time.sleep(0.01)

    def _place_ball(self):
        board = self._board_pose()
        p, q = board.pose.position, board.pose.orientation
        R = _quat_matrix(q.w, q.x, q.y, q.z)
        ball = np.array([p.x, p.y, p.z]) + R @ np.array([*self.cfg.ball.start, self.cfg.ball.radius])
        state = FreeJointState(name="ball")
        state.pose.header.frame_id = "world"
        state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z = (float(v) for v in ball)
        state.pose.pose.orientation.w = 1.0
        result = self._call(self.ball_cli, SetFreeJointState.Request(free_joints=[state]))
        if not result.success:
            raise RuntimeError(f"set_free_joint_state failed: {result.message}")

    def _board_pose(self, timeout=CALL_TIMEOUT_S):
        """One fresh board pose. Subscribed only for the moment it is needed, so this node takes
        no part in the 100 Hz pose stream."""
        got, done = [], threading.Event()

        def on_pose(msg):
            got.append(msg)
            done.set()

        sub = self.create_subscription(PoseStamped, BOARD_POSE_TOPIC, on_pose, 1, callback_group=self.group)
        try:
            if not done.wait(timeout):
                raise RuntimeError(f"no board pose on {BOARD_POSE_TOPIC} within {timeout} s")
            return got[-1]
        finally:
            self.destroy_subscription(sub)

    def _call(self, client, request, timeout=CALL_TIMEOUT_S):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise RuntimeError(f"{client.srv_name} did not answer within {timeout} s")
        return future.result()

    def _switch(self, activate, deactivate):
        if not activate and not deactivate:
            return
        req = SwitchController.Request(
            activate_controllers=list(activate), deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT,
        )
        if not self._call(self.switch_cli, req).ok:
            raise RuntimeError(f"switch_controllers failed (activate {activate}, deactivate {deactivate})")


def rtf_message(rtf, target):
    """Log level and text for a measured real-time factor against physics.real_time_factor."""
    text = f"real-time factor: {rtf:.2f}"
    if rtf >= 0.95 * target:
        return "info", text
    return "warn", (
        f"{text} (target {target:.2f}): sim time runs slower than asked. Nodes on the wall clock, "
        "as lab code is, see the arm move less per second than on the robot; close other programs "
        "or launch with camera:=false"
    )


def _quat_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main(args=None):
    rclpy.init(args=args)
    node = SimTools()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Shared pieces of the launch tests: launching the sim and talking to the controller manager."""

import os
import time

import launch
import launch_testing
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from launch.launch_description_sources import PythonLaunchDescriptionSource

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
STARTUP_S = 60.0
STARTUP_STATES = {
    "scaled_joint_trajectory_controller": "active",
    "joint_state_broadcaster": "active",
    "gripper_controller": "active",
    "forward_velocity_controller": "inactive",
    "forward_position_controller": "inactive",
}


def sim_launch_description(**args):
    sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("maze_sim_bringup"), "launch", "sim.launch.py")
        ),
        launch_arguments={"viewer": "false", "camera": "false", **args}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        value = predicate()
        if value:
            return value
    return predicate()


def _call(node, srv_type, name, request):
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        return future.result()
    finally:
        node.destroy_client(client)


def controller_states(node):
    result = _call(node, ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
    return {c.name: c.state for c in result.controller} if result else {}


def wait_for_startup(node):
    """Controller states once the spawners are done and sim_tools has run its start-up reset
    (or whatever they are after STARTUP_S)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Empty

    done = []
    sub = node.create_subscription(
        Empty, "/sim/reset_done", done.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )
    ready = lambda: done and (lambda s: s if all(s.get(k) == v for k, v in STARTUP_STATES.items()) else None)(
        controller_states(node)
    )
    states = spin_until(node, ready, STARTUP_S) or controller_states(node)
    node.destroy_subscription(sub)
    return states


def switch(node, activate, deactivate):
    req = SwitchController.Request(
        activate_controllers=list(activate), deactivate_controllers=list(deactivate),
        strictness=SwitchController.Request.STRICT,
    )
    result = _call(node, SwitchController, "/controller_manager/switch_controller", req)
    return bool(result and result.ok)


def default_config():
    from maze_sim_description.config import load_sim_config

    return load_sim_config(os.path.join(get_package_share_directory("maze_sim_description"), "config", "sim.yaml"))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose_matrix(pose):
    """4x4 transform of a geometry_msgs/Pose."""
    import numpy as np

    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T

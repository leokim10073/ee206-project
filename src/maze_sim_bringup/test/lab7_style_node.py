#!/usr/bin/env python3
"""Written the way the lab 7 starter code drives the real UR7e: switch to the forward velocity
controller with `ros2 control switch_controllers`, read /joint_states, publish joint velocities.
Turns wrist_3 at 0.1 rad/s for 2 s, then stops. Nothing here knows about the simulator."""

import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def switch_controllers(activate, deactivate):
    subprocess.run(
        ["ros2", "control", "switch_controllers", "--activate", activate, "--deactivate", deactivate, "--strict"],
        check=True,
    )


class Lab7StyleNode(Node):
    def __init__(self):
        super().__init__("lab7_style_node")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)

    def on_joint_state(self, msg):
        self.joint_state = msg

    def publish(self, velocities):
        self.pub.publish(Float64MultiArray(data=velocities))


def main():
    rclpy.init()
    node = Lab7StyleNode()
    while node.joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    switch_controllers("forward_velocity_controller", "scaled_joint_trajectory_controller")
    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds < 2e9:
        node.publish([0.0, 0.0, 0.0, 0.0, 0.0, 0.1])
        rclpy.spin_once(node, timeout_sec=0.01)
    node.publish([0.0] * 6)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""/toggle_gripper (std_srvs/Trigger), as in lab 5: each call opens a closed gripper or closes an
open one and answers "opened" or "closed". Starts closed (the board is held from the first
frame) and goes back to closed on every /sim/reset_done, since the reset closes the fingers.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, Float64MultiArray
from std_srvs.srv import Trigger

from maze_sim_bringup.sim_tools_node import GRIPPER_CLOSED

GRIPPER_OPEN = 0.0


class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")
        self.closed = True
        self.pub = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Empty, "/sim/reset_done", self.on_reset_done, latched)
        self.create_service(Trigger, "/toggle_gripper", self.on_toggle)

    def on_reset_done(self, _msg):
        self.closed = True

    def on_toggle(self, _request, response):
        self.closed = not self.closed
        self.pub.publish(Float64MultiArray(data=[GRIPPER_CLOSED if self.closed else GRIPPER_OPEN]))
        response.success, response.message = True, "closed" if self.closed else "opened"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

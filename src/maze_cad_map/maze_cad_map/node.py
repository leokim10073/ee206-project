"""cad_map: publishes a CAD level's maze map (contract 1, docs/interfaces.md), so planning can run
before perception P2 exists, and P2 has an answer key.

Publishes (reliable, transient local: a node that starts later still gets the map)
  /maze/map         maze_interfaces/MazeMap    the map
  /maze/map/walls   nav_msgs/OccupancyGrid     the same walls grid, for RViz

Parameters
  cad_scripts  path to the repo's maze_CAD/scripts (the launch file defaults to ./maze_CAD/scripts)
  level        key in levels.py, default L1
  resolution   m per cell, default 0.005

Never run it next to a perception map publisher: both publish /maze/map.
"""

from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from maze_cad_map.cad_map import DEFAULT_RESOLUTION, build_map
from maze_interfaces.msg import Circle, MazeMap

FRAME = "maze_board"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_msgs(cad_map, stamp):
    """(MazeMap, OccupancyGrid) for a CadMap"""
    grid = OccupancyGrid()
    grid.header.frame_id = FRAME
    grid.header.stamp = stamp
    grid.info.map_load_time = stamp
    grid.info.resolution = float(cad_map.resolution)
    grid.info.width, grid.info.height = cad_map.width, cad_map.height
    grid.info.origin.position.x, grid.info.origin.position.y = (float(v) for v in cad_map.origin)
    grid.info.origin.orientation.w = 1.0
    grid.data = cad_map.occupancy_data()

    msg = MazeMap(source=cad_map.source, walls=grid)
    msg.header.frame_id = FRAME
    msg.header.stamp = stamp
    msg.holes = [Circle(x=float(h.x), y=float(h.y), radius=float(h.radius)) for h in cad_map.holes]
    msg.start = Point(x=float(cad_map.start[0]), y=float(cad_map.start[1]), z=0.0)
    msg.goal = Circle(x=float(cad_map.goal.x), y=float(cad_map.goal.y), radius=float(cad_map.goal.radius))
    return msg, grid


class CadMapPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__("cad_map", **kwargs)
        self.declare_parameter("cad_scripts", str(Path.cwd() / "maze_CAD" / "scripts"))
        self.declare_parameter("level", "L1")
        self.declare_parameter("resolution", DEFAULT_RESOLUTION)
        p = self.get_parameter
        cad_scripts, level = p("cad_scripts").value, p("level").value
        cad_map = build_map(cad_scripts, level, float(p("resolution").value))

        self.map_pub = self.create_publisher(MazeMap, "/maze/map", LATCHED)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/maze/map/walls", LATCHED)
        msg, grid = to_msgs(cad_map, self.get_clock().now().to_msg())
        self.map_pub.publish(msg)
        self.grid_pub.publish(grid)
        self.get_logger().info(
            f"published {cad_map.source} from {cad_scripts}: {cad_map.width} x {cad_map.height} cells of "
            f"{1000 * cad_map.resolution:g} mm, start ({cad_map.start[0]:.3f}, {cad_map.start[1]:.3f}), "
            f"goal ({cad_map.goal.x:.3f}, {cad_map.goal.y:.3f}) r {cad_map.goal.radius:.3f}, "
            f"{len(cad_map.holes)} trap holes")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CadMapPublisher()
    except (FileNotFoundError, KeyError, ValueError) as e:
        rclpy.logging.get_logger("cad_map").fatal(str(e))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

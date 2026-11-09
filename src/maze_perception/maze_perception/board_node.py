"""board_tracker: the maze board's pose from the overhead camera's ArUco markers.

Subscribes
  image_topic        sensor_msgs/Image       (default /camera/image_raw; rgb8, bgr8, rgba8, bgra8 or mono8)
  camera_info_topic  sensor_msgs/CameraInfo  (default /camera/camera_info)

Publishes
  /perception/board_pose   geometry_msgs/PoseStamped  board frame in the image's frame (the camera
                           optical frame), stamped with the image; only for frames where it was found
  /perception/board_image  sensor_msgs/Image (rgb8)   rectified top-down board image (see
                           perception.yaml -> rectified), at most `rectified_rate` Hz
  TF  <image frame> -> `board_frame` (default maze_board), if `publish_tf`

Every 10 s it logs the frame rate, the share of frames with a board pose and the processing time.
"""

import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from maze_perception.board_pose import Camera, estimate_board_pose, rectify
from maze_perception.config import load_perception_config
from maze_perception.markers import MarkerDetector, opencv_version_warning
from maze_perception.ros_image import image_to_rgb, rgb_to_image

STATS_PERIOD_S = 10.0
# newest frame only: a late image is worth less than the next one
SENSOR_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

class BoardTracker(Node):
    def __init__(self, **kwargs):
        super().__init__("board_tracker", **kwargs)
        from ament_index_python.packages import get_package_share_directory

        default_cfg = Path(get_package_share_directory("maze_perception")) / "config" / "perception.yaml"
        self.declare_parameter("config", str(default_cfg))
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("board_frame", "maze_board")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rectified_rate", 5.0)

        p = self.get_parameter
        self.cfg = load_perception_config(p("config").value)
        self.board_frame = p("board_frame").value
        self.publish_tf = p("publish_tf").value
        rate = float(p("rectified_rate").value)
        self.rectified_period = 1.0 / rate if rate > 0 else None

        warning = opencv_version_warning()
        if warning:
            self.get_logger().warn(warning)
        self.detector = MarkerDetector(self.cfg.board.dictionary, ids=self.cfg.board.markers)
        self.camera = None
        self.last_rectified = None

        self.pose_pub = self.create_publisher(PoseStamped, "/perception/board_pose", 10)
        self.image_pub = self.create_publisher(Image, "/perception/board_image", 1)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(CameraInfo, p("camera_info_topic").value, self._on_info, SENSOR_QOS)
        self.create_subscription(Image, p("image_topic").value, self._on_image, SENSOR_QOS)

        self._stats = {"frames": 0, "found": 0, "busy_s": 0.0, "since": time.monotonic()}
        self.create_timer(STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(f"board_tracker on {p('image_topic').value}, ArUco {self.cfg.board.dictionary} "
                               f"ids {sorted(self.cfg.board.markers)}, min_markers {self.cfg.pose.min_markers}")

    def _on_info(self, msg: CameraInfo):
        if msg.distortion_model not in ("", "plumb_bob", "rational_polynomial"):
            self.get_logger().error(f"unsupported distortion model {msg.distortion_model!r}",
                                    throttle_duration_sec=10.0)
            return
        self.camera = Camera.from_camera_info(msg.k, msg.d, msg.width, msg.height)

    def _on_image(self, msg: Image):
        if self.camera is None:
            self.get_logger().warn("no camera_info yet", throttle_duration_sec=5.0)
            return
        t0 = time.monotonic()
        try:
            img = image_to_rgb(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        corners = self.detector.detect(img)
        pose = estimate_board_pose(corners, self.cfg.board, self.camera, self.cfg.pose)
        self._stats["frames"] += 1
        if pose is None:
            self.get_logger().warn(f"board not found: markers {sorted(corners)} seen, "
                                   f"need {self.cfg.pose.min_markers}", throttle_duration_sec=2.0)
        else:
            self._stats["found"] += 1
            self._publish_pose(msg.header, pose)
            now = time.monotonic()
            if self.rectified_period is not None and (
                    self.last_rectified is None or now - self.last_rectified >= self.rectified_period):
                self.last_rectified = now
                self.image_pub.publish(rgb_to_image(rectify(img, pose, self.camera, self.cfg.rectified), msg.header))
        self._stats["busy_s"] += time.monotonic() - t0

    def _publish_pose(self, header, pose):
        x, y, z, w = pose.quaternion()
        out = PoseStamped()
        out.header = header
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (float(v) for v in pose.t)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = x, y, z, w
        self.pose_pub.publish(out)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header = header
            tf.child_frame_id = self.board_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
                float(v) for v in pose.t)
            tf.transform.rotation = out.pose.orientation
            self.tf.sendTransform(tf)

    def _log_stats(self):
        s = self._stats
        dt = time.monotonic() - s["since"]
        if s["frames"]:
            self.get_logger().info(
                f"{s['frames'] / dt:.1f} frames/s (wall clock), board found in {100 * s['found'] / s['frames']:.0f}%, "
                f"{1000 * s['busy_s'] / s['frames']:.1f} ms per frame")
        else:
            self.get_logger().warn("no camera images received")
        self._stats = {"frames": 0, "found": 0, "busy_s": 0.0, "since": time.monotonic()}


def main(args=None):
    rclpy.init(args=args)
    node = BoardTracker()
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

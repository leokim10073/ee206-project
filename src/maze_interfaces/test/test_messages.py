"""Every maze_interfaces message builds, survives serialization, and goes over ROS with the QoS
docs/interfaces.md gives it. Needs ROS 2 and a built workspace (inside maze-sim:
colcon build && source install/setup.bash); skipped without rclpy."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Point, PoseStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402

from maze_interfaces.msg import BallState, BoardTilt, Circle, Corridor, MazeMap, PlannedRoute  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def maze_map():
    m = MazeMap(source="cad:L1")
    m.header.frame_id = "maze_board"
    m.walls.header.frame_id = "maze_board"
    m.walls.info.resolution = 0.005
    m.walls.info.width, m.walls.info.height = 3, 2
    m.walls.info.origin.position.x = m.walls.info.origin.position.y = -0.15
    m.walls.info.origin.orientation.w = 1.0
    m.walls.data = [100, 0, 0, 100, -1, 0]
    m.holes = [Circle(x=0.05, y=0.0, radius=0.01)]
    m.start = Point(x=-0.12, y=0.12)
    m.goal = Circle(x=0.12, y=-0.072, radius=0.015)
    return m


def ball_state():
    b = BallState(x=-0.12, y=0.12, vx=0.01, vy=-0.02, position_covariance=[1e-6, 0.0, 0.0, 1e-6], tracking_ok=True)
    b.header.frame_id = "maze_board"
    return b


def planned_route():
    r = PlannedRoute(plan_id=3, status=PlannedRoute.STATUS_OK)
    r.header.frame_id = r.path.header.frame_id = "maze_board"
    for x, y in [(-0.12, 0.12), (-0.06, 0.12), (-0.06, 0.06)]:
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        r.path.poses.append(p)
    r.arc_length = [0.0, 0.06, 0.12]
    r.corridors = [Corridor(x_min=-0.13, x_max=-0.05, y_min=0.11, y_max=0.13),
                   Corridor(x_min=-0.07, x_max=-0.05, y_min=0.05, y_max=0.13)]
    r.pose_corridor = [0, 0, 1]
    return r


def board_tilt():
    t = BoardTilt(tilt_x=0.02, tilt_y=-0.01)
    t.header.frame_id = "maze_board"
    return t


MESSAGES = {"map": maze_map, "ball": ball_state, "route": planned_route, "tilt": board_tilt}


@pytest.mark.parametrize("make", MESSAGES.values(), ids=MESSAGES.keys())
def test_serialization_round_trip(make):
    msg = make()
    wire = serialize_message(msg)
    back = deserialize_message(wire, type(msg))
    # compare on the wire: OccupancyGrid's resolution is float32, so 0.005 does not round-trip as a Python float
    assert serialize_message(back) == wire
    assert back.header.frame_id == "maze_board"


def test_constants_and_fixed_sizes():
    assert PlannedRoute.STATUS_OK == 0 and PlannedRoute.STATUS_NO_PATH == 1
    with pytest.raises(AssertionError):
        BallState(position_covariance=[0.0, 0.0, 0.0])  # exactly 4 entries


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pub_sub_with_documented_qos(ros):
    """Each type over a topic named as in docs/interfaces.md. The map and route subscribers join
    after the message was published and must still get it (transient local)."""
    pub_node = rclpy.create_node("interfaces_test_pub")
    sub_node = rclpy.create_node("interfaces_test_sub")
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    latched = {"/maze/map": maze_map(), "/maze/route": planned_route()}
    streamed = {"/ball/state": ball_state(), "/board/tilt_cmd": board_tilt()}
    pubs = {t: pub_node.create_publisher(type(m), t, LATCHED) for t, m in latched.items()}
    for topic, msg in latched.items():
        pubs[topic].publish(msg)
    stream_pubs = {t: pub_node.create_publisher(type(m), t, 10) for t, m in streamed.items()}

    got = {}
    for topic, msg in latched.items():
        sub_node.create_subscription(type(msg), topic, lambda m, t=topic: got.setdefault(t, m), LATCHED)
    for topic, msg in streamed.items():
        sub_node.create_subscription(type(msg), topic, lambda m, t=topic: got.setdefault(t, m), 10)

    deadline = time.monotonic() + 10.0
    while len(got) < 4 and time.monotonic() < deadline:
        for topic, msg in streamed.items():
            stream_pubs[topic].publish(msg)
        executor.spin_once(timeout_sec=0.05)
    pub_node.destroy_node()
    sub_node.destroy_node()

    for topic, msg in {**latched, **streamed}.items():
        assert topic in got, f"nothing received on {topic}"
        assert serialize_message(got[topic]) == serialize_message(msg), f"{topic} not received intact"

"""cad_map over ROS: a subscriber that starts after the map was published still receives it.
Needs ROS 2 and a built workspace (inside maze-sim); skipped without rclpy."""

import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from maze_cad_map.node import LATCHED, CadMapPublisher  # noqa: E402
from maze_interfaces.msg import MazeMap  # noqa: E402

CAD = Path(__file__).resolve().parents[3] / "maze_CAD" / "scripts"


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_late_subscriber_gets_the_map(ros):
    publisher = CadMapPublisher(parameter_overrides=[Parameter("cad_scripts", value=str(CAD))])
    peer = rclpy.create_node("cad_map_test")
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(peer)
    for _ in range(10):  # the map is already out before anyone listens
        executor.spin_once(timeout_sec=0.02)

    maps, grids = [], []
    peer.create_subscription(MazeMap, "/maze/map", maps.append, LATCHED)
    peer.create_subscription(OccupancyGrid, "/maze/map/walls", grids.append, LATCHED)
    deadline = time.monotonic() + 10.0
    while (not maps or not grids) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    publisher.destroy_node()
    peer.destroy_node()

    assert maps and grids
    m = maps[0]
    assert m.header.frame_id == "maze_board" and m.walls.header.frame_id == "maze_board"
    assert m.source == "cad:L1"
    assert (m.walls.info.width, m.walls.info.height) == (60, 60)
    assert m.walls.info.resolution == pytest.approx(0.005)
    assert (m.walls.info.origin.position.x, m.walls.info.origin.position.y) == pytest.approx((-0.15, -0.15))
    assert len(m.walls.data) == 3600 and set(m.walls.data) == {0, 100}
    assert (m.start.x, m.start.y) == pytest.approx((-0.12, 0.12))
    assert (m.goal.x, m.goal.y, m.goal.radius) == pytest.approx((0.12, -0.072, 0.015))
    assert list(grids[0].data) == list(m.walls.data)


def test_missing_cad_scripts_fails_clearly(ros, tmp_path):
    with pytest.raises(FileNotFoundError, match="maze_params.py"):
        CadMapPublisher(parameter_overrides=[Parameter("cad_scripts", value=str(tmp_path))])

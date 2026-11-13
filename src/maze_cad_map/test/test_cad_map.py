"""The CAD map against the other copies of the same maze: the sim export (parts_report.json) and
sim.yaml. Plain Python, no ROS."""

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from maze_cad_map.cad_map import FREE, WALL, build_map, path_cells_board

REPO = Path(__file__).resolve().parents[3]
CAD = REPO / "maze_CAD" / "scripts"
REPORT = json.loads((REPO / "maze_CAD" / "levels" / "L1" / "sim" / "parts_report.json").read_text())
SIM = yaml.safe_load((REPO / "src" / "maze_sim_description" / "config" / "sim.yaml").read_text())


@pytest.fixture(scope="module")
def cad_map():
    return build_map(CAD, "L1")


def test_grid_covers_the_plate(cad_map):
    assert (cad_map.height, cad_map.width) == (60, 60)
    assert cad_map.resolution == 0.005
    assert cad_map.origin == pytest.approx((-0.15, -0.15))
    assert cad_map.grid.dtype == np.int8 and set(np.unique(cad_map.grid)) == {FREE, WALL}


def test_start_and_goal_match_the_sim_export(cad_map):
    assert cad_map.start == pytest.approx(REPORT["ball_start_m"])
    assert cad_map.start == pytest.approx(SIM["ball"]["start"])
    g = REPORT["goal_hole"]
    assert (cad_map.goal.x, cad_map.goal.y) == pytest.approx(g["center_m"])
    assert cad_map.goal.radius == pytest.approx(g["radius_m"])
    assert (cad_map.goal.x, cad_map.goal.y) == pytest.approx(SIM["board"]["goal_hole"]["center"])
    assert cad_map.holes == []  # L1 has no trap holes


def test_every_exported_wall_is_in_the_grid(cad_map):
    """Each wall box of the sim export (maze_board, mm) covers wall cells and only wall cells."""
    for name, label in REPORT["wall_names"].items():
        if label == "goal_pocket_floor":
            continue
        lo, hi = REPORT["files"][f"{name}.stl"]["min_mm"], REPORT["files"][f"{name}.stl"]["max_mm"]
        r0, c0 = cad_map.cell_of(lo[0] / 1000 + 1e-6, lo[1] / 1000 + 1e-6)
        r1, c1 = cad_map.cell_of(hi[0] / 1000 - 1e-6, hi[1] / 1000 - 1e-6)
        inner = [(r, c) for r in range(r0, r1 + 1) for c in range(c0, c1 + 1)
                 if lo[0] <= 1000 * cad_map.cell_center(r, c)[0] <= hi[0]
                 and lo[1] <= 1000 * cad_map.cell_center(r, c)[1] <= hi[1]]
        assert inner, f"{name} ({label}) has no cell center inside it"
        assert all(cad_map.grid[rc] == WALL for rc in inner), f"{name} ({label}) not all wall"


def test_rows_count_from_the_bottom(cad_map):
    """Wall 4 (h_line2) sits at y = +45..+51 mm, i.e. near the TOP; it must land in a high row."""
    lo = REPORT["files"]["wall_4.stl"]["min_mm"]
    row, _ = cad_map.cell_of(-0.10, lo[1] / 1000 + 0.003)
    assert row > cad_map.height // 2
    assert cad_map.grid[row, cad_map.cell_of(-0.10, 0.0)[1]] == WALL


def test_intended_route_start_and_goal_are_free(cad_map):
    for x, y in path_cells_board(CAD, "L1"):
        assert cad_map.grid[cad_map.cell_of(x, y)] == FREE
    assert cad_map.grid[cad_map.cell_of(*cad_map.start)] == FREE
    assert cad_map.grid[cad_map.cell_of(cad_map.goal.x, cad_map.goal.y)] == FREE


def test_outside_the_outer_walls_is_wall(cad_map):
    assert (cad_map.grid[0, :] == WALL).all() and (cad_map.grid[-1, :] == WALL).all()
    assert (cad_map.grid[:, 0] == WALL).all() and (cad_map.grid[:, -1] == WALL).all()


def test_occupancy_data_order(cad_map):
    data = cad_map.occupancy_data()
    assert len(data) == 3600
    r, c = cad_map.cell_of(*cad_map.start)
    assert data[r * cad_map.width + c] == FREE


def test_bad_inputs(tmp_path):
    with pytest.raises(FileNotFoundError):
        build_map(tmp_path, "L1")
    with pytest.raises(KeyError):
        build_map(CAD, "L99")
    with pytest.raises(ValueError):
        build_map(CAD, "L1", resolution=0.007)

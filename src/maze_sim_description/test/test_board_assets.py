import mujoco
import pytest
import trimesh

from conftest import wrap_minimal_model
from maze_sim_description.board_assets import board_mjcf_fragment, build_board_assets


def test_scales_mm_to_m(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    assert a.extent_m[0] == pytest.approx(0.5, abs=1e-3)


def test_wrong_units_refused(placeholder_mm, tmp_path):
    with pytest.raises(ValueError, match="units"):
        build_board_assets(placeholder_mm, tmp_path / "out", units="m")  # 500 m board


def test_walls_are_single_convex_pieces(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    wall_pieces = [p for p in a.collision_meshes if p.name.startswith("wall_")]
    assert len(wall_pieces) == 2  # one piece per wall file


def test_hole_stays_open(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    base = [trimesh.load(p) for p in a.collision_meshes if p.name.startswith("base_")]
    ray = dict(ray_origins=[[0.100, 0.100, 1.0]], ray_directions=[[0, 0, -1]])
    assert not any(m.ray.intersects_any(**ray)[0] for m in base)  # hole center is open
    ray["ray_origins"] = [[-0.100, -0.100, 1.0]]
    assert any(m.ray.intersects_any(**ray)[0] for m in base)  # solid plate elsewhere


def test_handle_center(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    assert a.handle_center_m == pytest.approx((-0.270, 0.0, -0.003), abs=1e-6)


def test_fragment_compiles(placeholder_mm, tmp_path):
    out = tmp_path / "out"
    a = build_board_assets(placeholder_mm, out, units="mm")
    frag = board_mjcf_fragment(a, mass=0.8, friction=(1.0, 0.005, 0.0001))
    m = mujoco.MjModel.from_xml_string(wrap_minimal_model(frag, meshdir=out))
    assert m.body("board").mass[0] == pytest.approx(0.8)


def test_load_matches_build(placeholder_mm, tmp_path):
    from maze_sim_description.board_assets import load_board_assets

    built = build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    loaded = load_board_assets(tmp_path / "out")
    assert sorted(loaded.collision_meshes) == sorted(built.collision_meshes)
    assert sorted(loaded.visual_meshes) == sorted(built.visual_meshes)
    assert loaded.extent_m == pytest.approx(built.extent_m)
    assert loaded.handle_center_m == pytest.approx(built.handle_center_m, abs=1e-9)

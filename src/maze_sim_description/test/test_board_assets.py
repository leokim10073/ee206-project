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


def test_rebuild_removes_old_pieces(placeholder_mm, tmp_path):
    # a previous board split into more pieces / had more walls than the new one
    from maze_sim_description.board_assets import load_board_assets

    out = tmp_path / "out"
    build_board_assets(placeholder_mm, out, units="mm")
    for stale in ("base_009.obj", "wall_7.obj", "visual_wall_7.obj"):
        (out / "base_000.obj").rename(out / stale) if stale == "base_009.obj" else (out / stale).write_text("")
    build_board_assets(placeholder_mm, out, units="mm")
    loaded = load_board_assets(out)
    names = {p.name for p in loaded.collision_meshes + loaded.visual_meshes}
    assert not {"base_009.obj", "wall_7.obj", "visual_wall_7.obj"} & names
    assert not any((out / n).exists() for n in ("base_009.obj", "wall_7.obj", "visual_wall_7.obj"))


def test_load_refuses_units_other_than_config(placeholder_mm, tmp_path):
    from maze_sim_description.board_assets import load_board_assets

    build_board_assets(placeholder_mm, tmp_path / "out", units="mm")
    with pytest.raises(ValueError, match="board.units"):
        load_board_assets(tmp_path / "out", units="m")


def test_load_refuses_wrong_size(placeholder_mm, tmp_path):
    from maze_sim_description.board_assets import load_board_assets

    out = tmp_path / "out"
    build_board_assets(placeholder_mm, out, units="mm")
    base = trimesh.load(out / "visual_base.obj", force="mesh")
    base.apply_scale(1000.0)  # meshes copied in from an export that skipped the mm -> m scale
    base.export(out / "visual_base.obj")
    with pytest.raises(ValueError, match="board.units"):
        load_board_assets(out)


def test_load_needs_manifest(placeholder_mm, tmp_path):
    from maze_sim_description.board_assets import load_board_assets

    out = tmp_path / "out"
    build_board_assets(placeholder_mm, out, units="mm")
    (out / "board.json").unlink()
    with pytest.raises(FileNotFoundError, match="build_board_assets"):
        load_board_assets(out)


def test_cli_requires_out_dir(placeholder_mm, monkeypatch):
    # an installed console script has no reliable path back to src/, so the output is explicit
    from maze_sim_description import board_assets

    monkeypatch.setattr("sys.argv", ["build_board_assets", str(placeholder_mm), "--units", "mm"])
    with pytest.raises(SystemExit):
        board_assets.main()

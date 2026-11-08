"""Turn the board's STL parts into MuJoCo meshes.

Input (spec section 6): base.stl, handle.stl and wall_*.stl in one frame, origin at the
board center, z up. Walls are boxes, so each is one convex hull. The base (with holes) and
the handle are split into convex pieces with CoACD so holes stay open.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import coacd
import numpy as np
import trimesh

UNIT_SCALE = {"mm": 0.001, "m": 1.0}
EXTENT_RANGE_M = (0.2, 1.0)
COACD_THRESHOLD = 0.02


@dataclass(frozen=True)
class BoardAssets:
    visual_meshes: list
    collision_meshes: list
    extent_m: tuple
    handle_center_m: tuple


def _load(path, scale):
    mesh = trimesh.load(path, force="mesh")
    mesh.apply_scale(scale)
    return mesh


def _decompose(mesh, prefix, out_dir):
    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=COACD_THRESHOLD)
    paths = []
    for i, (v, f) in enumerate(parts):
        p = out_dir / f"{prefix}_{i:03d}.obj"
        trimesh.Trimesh(v, f).export(p)
        paths.append(p)
    return paths


def build_board_assets(src_dir, out_dir, units):
    src_dir, out_dir = Path(src_dir), Path(out_dir)
    if units not in UNIT_SCALE:
        raise ValueError(f"board.units: expected mm or m, got {units!r}")
    scale = UNIT_SCALE[units]
    walls = sorted(src_dir.glob("wall_*.stl"))
    for required in ("base.stl", "handle.stl"):
        if not (src_dir / required).exists():
            raise FileNotFoundError(f"{src_dir / required} is missing")

    base = _load(src_dir / "base.stl", scale)
    extent = tuple(float(x) for x in base.extents)
    if not EXTENT_RANGE_M[0] <= max(extent) <= EXTENT_RANGE_M[1]:
        raise ValueError(
            f"board.units: base is {max(extent):.3g} m across with units={units!r}; "
            f"expected {EXTENT_RANGE_M[0]}–{EXTENT_RANGE_M[1]} m. Check the STL export units."
        )
    handle = _load(src_dir / "handle.stl", scale)

    out_dir.mkdir(parents=True, exist_ok=True)
    visual, collision = [], []
    for name, mesh in [("base", base), ("handle", handle)]:
        p = out_dir / f"visual_{name}.obj"
        mesh.export(p)
        visual.append(p)
        collision += _decompose(mesh, name, out_dir)
    for wall_path in walls:
        mesh = _load(wall_path, scale)
        p = out_dir / f"visual_{wall_path.stem}.obj"
        mesh.export(p)
        visual.append(p)
        hull = out_dir / f"{wall_path.stem}.obj"
        mesh.convex_hull.export(hull)
        collision.append(hull)

    return BoardAssets(
        visual_meshes=visual,
        collision_meshes=collision,
        extent_m=extent,
        handle_center_m=tuple(float(x) for x in handle.bounds.mean(axis=0)),
    )


def load_board_assets(mesh_dir):
    """BoardAssets for a directory that build_board_assets already wrote."""
    mesh_dir = Path(mesh_dir)
    visual = sorted(mesh_dir.glob("visual_*.obj"))
    collision = sorted(p for p in mesh_dir.glob("*.obj") if not p.name.startswith("visual_"))
    for required in ("visual_base.obj", "visual_handle.obj"):
        if not (mesh_dir / required).exists():
            raise FileNotFoundError(f"{mesh_dir / required} is missing; run build_board_assets first")
    base = trimesh.load(mesh_dir / "visual_base.obj", force="mesh")
    handle = trimesh.load(mesh_dir / "visual_handle.obj", force="mesh")
    return BoardAssets(
        visual_meshes=visual,
        collision_meshes=collision,
        extent_m=tuple(float(x) for x in base.extents),
        handle_center_m=tuple(float(x) for x in handle.bounds.mean(axis=0)),
    )


def _plate_inertia(mass, extent):
    x, y, z = extent
    return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)


def board_mjcf_fragment(assets, mass, friction):
    """<asset> and <worldbody> with a free body "board": visual geoms (group 1, no contact)
    and one collision geom per convex piece (group 3). Mesh files resolve against meshdir."""
    assets_xml, geoms = [], []
    for p in assets.visual_meshes:
        name = f"board_{p.stem}"
        assets_xml.append(f'<mesh name="{name}" file="{p.name}"/>')
        geoms.append(f'<geom type="mesh" mesh="{name}" group="1" contype="0" conaffinity="0" rgba="0.82 0.72 0.55 1"/>')
    fr = " ".join(f"{v:g}" for v in friction)
    for p in assets.collision_meshes:
        name = f"board_col_{p.stem}"
        assets_xml.append(f'<mesh name="{name}" file="{p.name}"/>')
        geoms.append(f'<geom name="{name}" type="mesh" mesh="{name}" group="3" friction="{fr}" condim="6"/>')
    ixx, iyy, izz = _plate_inertia(mass, assets.extent_m)
    return (
        "<asset>" + "".join(assets_xml) + "</asset>"
        '<worldbody><body name="board"><freejoint name="board_free"/>'
        f'<inertial pos="0 0 0" mass="{mass:g}" diaginertia="{ixx:g} {iyy:g} {izz:g}"/>'
        + "".join(geoms) + "</body></worldbody>"
    )


def main():
    p = argparse.ArgumentParser(description="Convert board STL parts into MuJoCo meshes.")
    p.add_argument("src_dir", help="directory with base.stl, handle.stl, wall_*.stl")
    p.add_argument("--units", required=True, choices=sorted(UNIT_SCALE))
    p.add_argument("--out-dir", default=str(Path(__file__).resolve().parents[1] / "meshes" / "board"))
    a = p.parse_args()
    assets = build_board_assets(a.src_dir, a.out_dir, a.units)
    print(f"{len(assets.visual_meshes)} visual, {len(assets.collision_meshes)} collision meshes → {a.out_dir}")


if __name__ == "__main__":
    main()

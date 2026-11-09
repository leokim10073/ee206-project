"""Turn the board's STL parts into MuJoCo meshes.

Input (spec section 6): base.stl, handle.stl and wall_*.stl in one frame, origin at the
board center, z up. Walls are boxes, so each is one convex hull. The base (with holes) and
the handle are split into convex pieces with CoACD so holes stay open. Optional:
aruco/aruco_<corner>.obj + .png, textured sticker slabs in the same frame and units; they are
drawn but never collide.

Output: the meshes plus board.json, which lists them and records the STL units. The output
directory belongs to this tool: every *.obj and marker *.png in it is replaced on each run, so
pieces of an earlier board never linger as invisible collision geometry.
"""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import coacd
import numpy as np
import trimesh

UNIT_SCALE = {"mm": 0.001, "m": 1.0}
EXTENT_RANGE_M = (0.2, 1.0)
COACD_THRESHOLD = 0.02
MANIFEST = "board.json"


@dataclass(frozen=True)
class BoardAssets:
    visual_meshes: list
    collision_meshes: list
    extent_m: tuple
    handle_center_m: tuple
    markers: list = ()  # textured visual-only meshes, each with a .png of the same stem


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


def _copy_marker(obj, out_dir, scale):
    """The sticker OBJ with its vertices scaled to metres (texture coordinates and faces kept),
    and its texture next to it."""
    lines = []
    for line in obj.read_text().splitlines():
        if line.startswith("v "):
            line = "v " + " ".join(f"{float(x) * scale:.9g}" for x in line.split()[1:4])
        lines.append(line)
    dst = out_dir / obj.name
    dst.write_text("\n".join(lines) + "\n")
    dst.with_suffix(".png").write_bytes(obj.with_suffix(".png").read_bytes())
    return dst


def _check_extent(extent, units, where):
    if not EXTENT_RANGE_M[0] <= max(extent) <= EXTENT_RANGE_M[1]:
        raise ValueError(
            f"board.units: {where} is {max(extent):.3g} m across with units={units!r}; "
            f"expected {EXTENT_RANGE_M[0]}–{EXTENT_RANGE_M[1]} m. Check the STL export units."
        )


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
    _check_extent(extent, units, "base")
    handle = _load(src_dir / "handle.stl", scale)

    out_dir.mkdir(parents=True, exist_ok=True)
    for old in [*out_dir.glob("*.obj"), *out_dir.glob("aruco_*.png"), out_dir / MANIFEST]:
        old.unlink(missing_ok=True)
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
    markers = [_copy_marker(p, out_dir, scale) for p in sorted((src_dir / "aruco").glob("aruco_*.obj"))]

    manifest = {
        "units": units,
        "visual": [p.name for p in visual],
        "collision": [p.name for p in collision],
        "markers": [p.name for p in markers],
    }
    (out_dir / MANIFEST).write_text(json.dumps(manifest, indent=2) + "\n")
    return BoardAssets(
        visual_meshes=visual,
        collision_meshes=collision,
        extent_m=extent,
        handle_center_m=tuple(float(x) for x in handle.bounds.mean(axis=0)),
        markers=markers,
    )


def load_board_assets(mesh_dir, units=None):
    """BoardAssets for a directory that build_board_assets already wrote: only the meshes its
    board.json lists. `units` (sim.yaml board.units), if given, must match the units the meshes
    were built from."""
    mesh_dir = Path(mesh_dir)
    if not (mesh_dir / MANIFEST).exists():
        raise FileNotFoundError(f"{mesh_dir / MANIFEST} is missing; run build_board_assets first")
    manifest = json.loads((mesh_dir / MANIFEST).read_text())
    if units is not None and units != manifest["units"]:
        raise ValueError(
            f"board.units: sim.yaml says {units!r} but the meshes in {mesh_dir} were built from "
            f"{manifest['units']!r} STLs; rerun build_board_assets with --units {units}"
        )
    visual = [mesh_dir / n for n in manifest["visual"]]
    collision = [mesh_dir / n for n in manifest["collision"]]
    markers = [mesh_dir / n for n in manifest.get("markers", [])]
    for p in visual + collision + markers + [m.with_suffix(".png") for m in markers]:
        if not p.exists():
            raise FileNotFoundError(f"{p} is listed in {MANIFEST} but missing; rerun build_board_assets")
    base = trimesh.load(mesh_dir / "visual_base.obj", force="mesh")
    handle = trimesh.load(mesh_dir / "visual_handle.obj", force="mesh")
    extent = tuple(float(x) for x in base.extents)
    _check_extent(extent, manifest["units"], f"the base in {mesh_dir}")
    return BoardAssets(
        visual_meshes=visual,
        collision_meshes=collision,
        extent_m=extent,
        handle_center_m=tuple(float(x) for x in handle.bounds.mean(axis=0)),
        markers=markers,
    )


def _plate_inertia(mass, extent):
    x, y, z = extent
    return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)


def board_mjcf_fragment(assets, mass, friction):
    """<asset> and <worldbody> with a free body "board": visual geoms (group 1, no contact)
    and one collision geom per convex piece (group 3), plus the textured markers (group 1, no
    contact). Mesh files resolve against meshdir, marker textures against texturedir."""
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
    for p in assets.markers:
        name = f"board_{p.stem}"
        assets_xml.append(
            f'<texture name="{name}" type="2d" file="{p.stem}.png"/>'
            f'<material name="{name}" texture="{name}"/><mesh name="{name}" file="{p.name}"/>'
        )
        geoms.append(f'<geom name="{name}" type="mesh" mesh="{name}" material="{name}" group="1" contype="0" conaffinity="0"/>')
    ixx, iyy, izz = _plate_inertia(mass, assets.extent_m)
    return (
        "<asset>" + "".join(assets_xml) + "</asset>"
        '<worldbody><body name="board"><freejoint name="board_free"/>'
        f'<inertial pos="0 0 0" mass="{mass:g}" diaginertia="{ixx:g} {iyy:g} {izz:g}"/>'
        + "".join(geoms) + "</body></worldbody>"
    )


def main():
    p = argparse.ArgumentParser(description="Convert board STL parts into MuJoCo meshes.")
    p.add_argument("src_dir", help="directory with base.stl, handle.stl, wall_*.stl (and optionally aruco/)")
    p.add_argument("--units", required=True, choices=sorted(UNIT_SCALE))
    p.add_argument("--out-dir", required=True,
                   help="where the meshes go, e.g. src/maze_sim_description/meshes/board (then colcon build)")
    a = p.parse_args()
    assets = build_board_assets(a.src_dir, a.out_dir, a.units)
    print(f"{len(assets.visual_meshes)} visual, {len(assets.collision_meshes)} collision meshes, "
          f"{len(assets.markers)} markers → {a.out_dir}")


if __name__ == "__main__":
    main()

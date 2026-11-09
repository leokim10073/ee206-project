"""Export a maze level as separate STL parts for the MuJoCo sim.

    python3 maze_CAD/scripts/export_sim.py --level L1     # -> maze_CAD/levels/L1/sim/

Output (all in one frame, units mm):
  base.stl       floor plate + the four ArUco pads, with the start ring, any trap holes and a
                 through hole for the goal pocket cut in
  handle.stl     grip tab the 2F-85 clamps (top/bottom faces)
  wall_<k>.stl   one straight box per wall (outer frame included); corners overlap. The last
                 one is the goal pocket's floor, a disk that closes the bottom of the goal hole
  aruco/         aruco_<corner>.obj + .png: one textured quad per corner marker. The sim's
                 board_assets tool reads only base/handle/wall_*, so these need their own
                 geoms in the scene (see the README)
  parts_report.json  bounding boxes, goal hole, start, handle, ArUco poses, and the checks below

Sim frame (the sim's placeholder board uses the same): origin at the center of the 300x300
base plate, z = 0 on the maze floor (the plate's top face), z up, +x away from the handle
(handle on the -x edge), units mm.

Left out on purpose (print-only details): ArUco sticker recesses, finger-pad recesses on
the handle, rounded pad corners, and the part of the printed tab under the outer wall.

Checks (the script exits non-zero if any fails):
  every STL watertight with outward normals (positive volume);
  every wall's volume equals its convex hull's within 1 %;
  every gap between walls that do not touch is wider than MIN_CORRIDOR;
  the goal pocket, start ring and trap holes clear every wall and each other;
  the base's longest side is 200-1000 mm.

Needs trimesh, shapely, mapbox_earcut and opencv (all in the maze-sim container).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import shapely.geometry as sg
import trimesh
from shapely import affinity
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

sys.path.insert(0, str(Path(__file__).resolve().parent))
from aruco import write_sim_quads  # noqa: E402
from levels import LEVELS  # noqa: E402
from maze_params import ARUCO, P, cell_center, sticker_size, tag_pads, wall_rects_named  # noqa: E402

BALL_RADIUS = 7.0         # mm, sim.yaml ball.radius
MIN_CORRIDOR = 18.0       # mm, required gap between walls (> 2 * BALL_RADIUS, with margin)
EXTENT_RANGE = (200.0, 1000.0)
HULL_TOL = 0.01
CIRCLE_SEGMENTS = 48      # per circle; the goal pocket edge is within 0.03 mm of round
ARUCO_LIFT = 0.1          # mm, marker quads sit this far above the floor so they render on top
LEVELS_DIR = Path(__file__).resolve().parents[1] / "levels"


def plate_center():
    return (P["base"] / 2, P["base"] / 2)


def to_sim(geom):
    """plate frame (origin at the plate's bottom-left corner) -> sim frame xy"""
    cx, cy = plate_center()
    return affinity.translate(geom, -cx, -cy)


def box(xmin, xmax, ymin, ymax, zmin, zmax):
    b = trimesh.creation.box(extents=(xmax - xmin, ymax - ymin, zmax - zmin))
    b.apply_translation(((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2))
    return b


def extrude(poly, z0, z1):
    """watertight prism of a shapely polygon (holes allowed) between z0 and z1"""
    mesh = trimesh.creation.extrude_polygon(poly, z1 - z0)
    mesh.apply_translation((0, 0, z0))
    return mesh


def _rings(poly):
    poly = orient(poly, sign=1.0)  # exterior counter-clockwise: the polygon is on each ring's left
    return [poly.exterior, *poly.interiors]


def _side(ring, z0, z1, into_polygon):
    """quads along a closed ring between z0 and z1, as (vertices, faces). Normals point out of
    the polygon (a plate's outer wall), or into it (into_polygon: the wall of a recess)."""
    xy = np.asarray(ring.coords)[:-1]
    n = len(xy)
    v = np.vstack([np.c_[xy, np.full(n, z0)], np.c_[xy, np.full(n, z1)]])
    i = np.arange(n); j = (i + 1) % n
    f = np.vstack([np.c_[i, j, j + n], np.c_[i, j + n, i + n]])  # normal on the ring's right
    return v, (f[:, ::-1] if into_polygon else f)


def _cap(geom, z, up):
    """flat triangulated face at height z, normals +z (up) or -z; a MultiPolygon (e.g. the top
    face around the start ring, which leaves an island in the middle) is done part by part"""
    verts, faces, off = [], [], 0
    for poly in getattr(geom, "geoms", [geom]):
        v2, f = trimesh.creation.triangulate_polygon(poly, engine="earcut")
        a, b, c = v2[f[:, 0]], v2[f[:, 1]], v2[f[:, 2]]
        nz = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        flip = (nz < 0) if up else (nz > 0)
        f[flip] = f[flip][:, ::-1]
        verts.append(np.c_[v2, np.full(len(v2), z)]); faces.append(f + off); off += len(v2)
    return np.vstack(verts), np.vstack(faces)


def recessed_plate(outline, z_bottom, z_top, recesses):
    """Watertight plate between z_bottom and z_top with recesses cut down from the top face,
    built face by face (the sim container has no boolean backend and no networkx, so no
    trimesh booleans or fix_normals). recesses: [(polygon, depth)], non-overlapping and inside
    the outline; a depth >= the plate thickness cuts a through hole. Polygons may have holes
    (a ring recess is an annulus)."""
    t = z_top - z_bottom
    through = [poly for poly, d in recesses if d >= t]
    blind = [(poly, d) for poly, d in recesses if d < t]
    cut_all = unary_union([poly for poly, _ in recesses])
    pieces = [_cap(outline.difference(cut_all), z_top, up=True),
              _cap(outline.difference(unary_union(through)) if through else outline, z_bottom, up=False)]
    pieces += [_side(r, z_bottom, z_top, into_polygon=False) for r in _rings(outline)]
    for poly in through:
        pieces += [_side(r, z_bottom, z_top, into_polygon=True) for r in _rings(poly)]
    for poly, d in blind:
        pieces += [_side(r, z_top - d, z_top, into_polygon=True) for r in _rings(poly)]
        pieces.append(_cap(poly, z_top - d, up=True))
    verts, faces, off = [], [], 0
    for v, f in pieces:
        verts.append(v); faces.append(f + off); off += len(v)
    mesh = trimesh.Trimesh(np.vstack(verts), np.vstack(faces), process=False)
    mesh.merge_vertices(digits_vertex=6)
    mesh.update_faces(mesh.nondegenerate_faces())
    return mesh


def recess_shapes(L):
    """{name: (sim-frame polygon, depth)} for the goal pocket, start ring and trap holes"""
    seg = CIRCLE_SEGMENTS // 4
    gx, gy = cell_center(L, L["goal"]); sx, sy = cell_center(L, L["start"])
    r_in, r_out, ring_d = P["start_ring"]
    out = {
        # cut through the base; build_parts adds the pocket floor back as a separate convex part
        "goal_pocket": (to_sim(sg.Point(gx, gy).buffer(P["goal_d"] / 2, seg)), P["base_t"]),
        "start_ring": (to_sim(sg.Point(sx, sy).buffer(r_out, seg).difference(sg.Point(sx, sy).buffer(r_in, seg))),
                       ring_d),
    }
    for k, (r, c, d) in enumerate(L.get("holes", [])):
        hx, hy = cell_center(L, (r, c))
        out[f"trap_hole_{k}"] = (to_sim(sg.Point(hx, hy).buffer(d / 2, seg)), P["base_t"])
    return out


def build_parts(L):
    """{name: trimesh} in the sim frame, plus the 2D walls and recesses for the checks"""
    bt, wh, B = P["base_t"], P["wall_h"], P["base"]
    parts = {}

    # base: plate + ArUco pads, top face on z = 0, with the pockets, ring and holes cut in
    outline = [sg.box(0, 0, B, B)] + [sg.box(x, y, x + S, y + S) for x, y, S, _ in tag_pads().values()]
    recesses = recess_shapes(L)
    parts["base"] = recessed_plate(to_sim(unary_union(outline)), -bt, 0.0, list(recesses.values()))

    # handle: the part of the tab outside the plate, outer corners rounded in plan view;
    # its bottom is flush with the base bottom, as printed
    hl, hw, hh, f = P["handle_l"], P["handle_w"], P["handle_h"], P["tab_fillet"]
    tab = sg.box(-hl, B / 2 - hw / 2, 0, B / 2 + hw / 2)
    rounded = unary_union([
        sg.box(-hl + f, B / 2 - hw / 2, 0, B / 2 + hw / 2),
        sg.box(-hl, B / 2 - hw / 2 + f, 0, B / 2 + hw / 2 - f),
        sg.Point(-hl + f, B / 2 - hw / 2 + f).buffer(f, 16),
        sg.Point(-hl + f, B / 2 + hw / 2 - f).buffer(f, 16),
    ]).intersection(tab)
    parts["handle"] = extrude(to_sim(rounded), -bt, hh - bt)

    # walls: one straight box each, standing on the floor (z = 0)
    walls2d = []
    for k, (name, (x, y, w, h)) in enumerate(wall_rects_named(L)):
        rect = to_sim(sg.box(x, y, x + w, y + h))
        x0, y0, x1, y1 = rect.bounds
        parts[f"wall_{k}"] = box(x0, x1, y0, y1, 0.0, wh)
        walls2d.append((f"wall_{k}", name, rect))

    # goal pocket floor: CoACD fills a 3 mm pocket in the base back in (it is too shallow
    # next to a 416 mm plate), so the base gets a through hole and this convex disk closes
    # its bottom. The sim takes every wall_*.stl as its exact convex hull, so the pocket is
    # exact. It is numbered after the walls.
    parts[f"wall_{len(walls2d)}"] = extrude(recesses["goal_pocket"][0], -bt, P["goal_depth"] - bt)
    return parts, walls2d, recesses


def check(parts, walls2d, recesses):
    problems, wall_checks = [], {}
    for name, m in parts.items():
        if not m.is_watertight:
            problems.append(f"{name}: not watertight")
        if not m.is_winding_consistent or m.volume <= 0:
            problems.append(f"{name}: normals not consistently outward")
        if name.startswith("wall_"):
            hull = m.convex_hull.volume
            diff = abs(hull - m.volume) / m.volume
            wall_checks[name] = {"volume_mm3": round(float(m.volume), 1), "hull_volume_mm3": round(float(hull), 1),
                                 "diff_percent": round(float(100 * diff), 4)}
            if diff >= HULL_TOL:
                problems.append(f"{name}: volume differs from its convex hull by {100 * diff:.2f}% — split it")
    gaps = []
    for i in range(len(walls2d)):
        for j in range(i + 1, len(walls2d)):
            d = walls2d[i][2].distance(walls2d[j][2])
            if d > 1e-6:
                gaps.append((d, walls2d[i][0], walls2d[j][0]))
    min_gap = min(gaps)[0] if gaps else None
    if min_gap is not None and min_gap <= MIN_CORRIDOR:
        problems.append(f"narrowest wall gap {min_gap:.1f} mm <= {MIN_CORRIDOR} mm")
    names = list(recesses)
    for i, name in enumerate(names):
        poly = recesses[name][0]
        for wname, label, rect in walls2d:
            if poly.intersects(rect):
                problems.append(f"{name} overlaps {wname} ({label})")
        for other in names[i + 1:]:
            if poly.intersects(recesses[other][0]):
                problems.append(f"{name} overlaps {other}")
    extent = max(parts["base"].extents)
    if not EXTENT_RANGE[0] <= extent <= EXTENT_RANGE[1]:
        problems.append(f"base longest side {extent:.1f} mm outside {EXTENT_RANGE}")
    return problems, wall_checks, min_gap


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--level", default="L1", choices=sorted(LEVELS))
    ap.add_argument("--out-dir", default=None, help="default: maze_CAD/levels/<level>/sim")
    a = ap.parse_args()
    L = LEVELS[a.level]
    out = Path(a.out_dir) if a.out_dir else LEVELS_DIR / a.level / "sim"
    out.mkdir(parents=True, exist_ok=True)

    parts, walls2d, recesses = build_parts(L)
    problems, wall_checks, min_gap = check(parts, walls2d, recesses)

    cx, cy = plate_center()
    gx, gy = cell_center(L, L["goal"]); sx, sy = cell_center(L, L["start"])
    pads = {k: (c[0] - cx, c[1] - cy) for k, (*_, c) in tag_pads().items()}
    markers = write_sim_quads(out / "aruco", pads, ARUCO_LIFT)
    h = parts["handle"]
    report = {
        "level": L["name"],
        "frame": "origin at base-plate center, z=0 on the maze floor (plate top), z up, handle on -x, units mm",
        "files": {},
        "goal_hole": {"center_m": [round((gx - cx) / 1000, 4), round((gy - cy) / 1000, 4)],
                      "radius_m": P["goal_d"] / 2 / 1000,
                      "depth_m": P["goal_depth"] / 1000,
                      "note": "a through hole in base.stl closed by the goal_pocket_floor disk (see wall_names); "
                              "center and radius go in sim.yaml board.goal_hole"},
        "ball_start_m": [round((sx - cx) / 1000, 4), round((sy - cy) / 1000, 4)],
        "start_ring_mm": {"inner_r": P["start_ring"][0], "outer_r": P["start_ring"][1], "depth": P["start_ring"][2]},
        "floor_z_mm": 0.0,
        "handle": {"center_mm": [round(float(v), 2) for v in h.bounds.mean(axis=0)],
                   "size_mm": [round(float(v), 2) for v in h.extents]},
        "aruco": {"dictionary": ARUCO["dictionary"], "marker_mm": P["tag_marker"], "sticker_mm": sticker_size(),
                  "marker_up": "+y", "markers": markers},
        "wall_names": {**{f"wall_{k}": n for k, (n, _) in enumerate(wall_rects_named(L))},
                       f"wall_{len(walls2d)}": "goal_pocket_floor"},
        "wall_hull_check": wall_checks,
        "narrowest_wall_gap_mm": round(float(min_gap), 2) if min_gap is not None else None,
        "problems": problems,
    }
    for old in out.glob("*.stl"):
        old.unlink()  # walls removed from the design must not linger
    for name, m in parts.items():
        p = out / f"{name}.stl"
        m.export(p)
        lo, hi = m.bounds
        report["files"][p.name] = {"min_mm": [round(float(v), 2) for v in lo], "max_mm": [round(float(v), 2) for v in hi],
                                   "watertight": bool(m.is_watertight)}
    (out / "parts_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    print(f"{len(parts)} parts + {len(markers)} ArUco quads -> {out}")
    for fname, info in report["files"].items():
        print(f"  {fname:12s} min {info['min_mm']}  max {info['max_mm']}")
    print("goal_hole:", report["goal_hole"]["center_m"], "r =", report["goal_hole"]["radius_m"], "m")
    print("ball_start:", report["ball_start_m"], "m")
    print("narrowest wall gap:", report["narrowest_wall_gap_mm"], "mm")
    if problems:
        print("PROBLEMS:\n  " + "\n  ".join(problems))
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()

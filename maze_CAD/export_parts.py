"""Export a maze level as separate STL parts for the MuJoCo sim.

    python3 maze_CAD/export_parts.py            # level L1 -> maze_CAD/L1_parts/
    python3 maze_CAD/export_parts.py --level L1 --out-dir some/dir

Output (all in one frame, units mm):
  base.stl       floor plate + the four ArUco pads, with trap holes cut through
  handle.stl     grip tab the 2F-85 clamps (top/bottom faces)
  wall_<k>.stl   one straight box per wall (outer frame included); corners overlap
  parts_report.json  bounding boxes, goal hole, start, handle, and the checks below

Sim frame: origin at the center of the 300x300 base plate, z = 0 at the base bottom,
z up, +x away from the handle (handle on the -x edge), units mm.

Left out on purpose (print-only details that would only add CoACD pieces):
  goal pocket (reported as center + radius instead), start ring engraving,
  ArUco sticker recesses, finger-pad recesses on the handle, rounded pad corners,
  and the part of the printed tab that runs under the outer wall.

Checks (the script exits non-zero if any fails):
  every STL watertight with outward normals (positive volume);
  every wall's volume equals its convex hull's within 1 %;
  every gap between walls that do not touch is wider than MIN_CORRIDOR;
  the base's longest side is 200-1000 mm.

Needs only trimesh and shapely (the versions in docker/Containerfile).
"""

import argparse
import json
import sys
from pathlib import Path

import shapely.geometry as sg
import trimesh
from shapely.ops import unary_union

sys.path.insert(0, str(Path(__file__).resolve().parent))
from maze_params import LEVELS, P, cell_center, tag_pads, wall_rects_named  # noqa: E402

BALL_RADIUS = 7.0         # mm, sim.yaml ball.radius
MIN_CORRIDOR = 18.0       # mm, required gap between walls (> 2 * BALL_RADIUS, with margin)
EXTENT_RANGE = (200.0, 1000.0)
HULL_TOL = 0.01


def plate_center():
    return (P["base"] / 2, P["base"] / 2)


def shift_polygon(poly, dx, dy):
    from shapely import affinity
    return affinity.translate(poly, dx, dy)


def extrude(poly, z0, z1):
    """watertight prism of a shapely polygon (holes allowed) between z0 and z1"""
    mesh = trimesh.creation.extrude_polygon(poly, z1 - z0)
    mesh.apply_translation((0, 0, z0))
    mesh.fix_normals()
    return mesh


def box(xmin, xmax, ymin, ymax, zmin, zmax):
    b = trimesh.creation.box(extents=(xmax - xmin, ymax - ymin, zmax - zmin))
    b.apply_translation(((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2))
    return b


def build_parts(L):
    """{name: trimesh} in the sim frame, plus the 2D wall rectangles for the corridor check"""
    cx, cy = plate_center()
    bt, wh, B = P["base_t"], P["wall_h"], P["base"]
    parts = {}

    # base: plate + ArUco pads, minus trap holes
    outline = [sg.box(0, 0, B, B)]
    for x, y, S, _ in tag_pads().values():
        outline.append(sg.box(x, y, x + S, y + S))
    base2d = unary_union(outline)
    for r, c, d in L.get("holes", []):
        hx, hy = cell_center(L, (r, c))
        base2d = base2d.difference(sg.Point(hx, hy).buffer(d / 2, 32))
    parts["base"] = extrude(shift_polygon(base2d, -cx, -cy), 0.0, bt)

    # handle: the part of the tab outside the plate, outer corners rounded in plan view
    hl, hw, hh, f = P["handle_l"], P["handle_w"], P["handle_h"], P["tab_fillet"]
    tab = sg.box(-hl, B / 2 - hw / 2, 0, B / 2 + hw / 2)
    # round only the two -x corners: union of an inset box and corner circles
    rounded = unary_union([
        sg.box(-hl + f, B / 2 - hw / 2, 0, B / 2 + hw / 2),
        sg.box(-hl, B / 2 - hw / 2 + f, 0, B / 2 + hw / 2 - f),
        sg.Point(-hl + f, B / 2 - hw / 2 + f).buffer(f, 16),
        sg.Point(-hl + f, B / 2 + hw / 2 - f).buffer(f, 16),
    ]).intersection(tab)
    parts["handle"] = extrude(shift_polygon(rounded, -cx, -cy), 0.0, hh)

    # walls: one straight box each, bottom on the floor (z = base_t)
    walls2d = []
    for k, (name, (x, y, w, h)) in enumerate(wall_rects_named(L)):
        parts[f"wall_{k}"] = box(x - cx, x + w - cx, y - cy, y + h - cy, bt, bt + wh)
        walls2d.append((f"wall_{k}", name, sg.box(x - cx, y - cy, x + w - cx, y + h - cy)))
    return parts, walls2d


def check(parts, walls2d):
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
    extent = max(parts["base"].extents)
    if not EXTENT_RANGE[0] <= extent <= EXTENT_RANGE[1]:
        problems.append(f"base longest side {extent:.1f} mm outside {EXTENT_RANGE}")
    return problems, wall_checks, min_gap


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--level", default="L1", choices=sorted(LEVELS))
    ap.add_argument("--out-dir", default=None, help="default: maze_CAD/<level>_parts")
    a = ap.parse_args()
    L = LEVELS[a.level]
    out = Path(a.out_dir) if a.out_dir else Path(__file__).resolve().parent / f"{a.level}_parts"
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.stl"):
        old.unlink()  # walls removed from the design must not linger

    parts, walls2d = build_parts(L)
    problems, wall_checks, min_gap = check(parts, walls2d)

    cx, cy = plate_center()
    gx, gy = cell_center(L, L["goal"]); sx, sy = cell_center(L, L["start"])
    h = parts["handle"]
    report = {
        "level": L["name"],
        "frame": "origin at base-plate center, z=0 at base bottom, z up, handle on -x, units mm",
        "files": {},
        "goal_hole": {"center_m": [round((gx - cx) / 1000, 4), round((gy - cy) / 1000, 4)],
                      "radius_m": P["goal_d"] / 2 / 1000,
                      "note": f"printed as a {P['goal_d']:.0f} mm x {P['goal_depth']:.0f} mm deep pocket; not in base.stl"},
        "ball_start_m": [round((sx - cx) / 1000, 4), round((sy - cy) / 1000, 4)],
        "floor_z_mm": P["base_t"],
        "handle": {"center_mm": [round(float(v), 2) for v in h.bounds.mean(axis=0)],
                   "size_mm": [round(float(v), 2) for v in h.extents]},
        "aruco_centers_mm": {k: [c[0] - cx, c[1] - cy, P["base_t"]] for k, (*_, c) in tag_pads().items()},
        "wall_names": {f"wall_{k}": n for k, (n, _) in enumerate(wall_rects_named(L))},
        "wall_hull_check": wall_checks,
        "narrowest_wall_gap_mm": round(float(min_gap), 2) if min_gap is not None else None,
        "problems": problems,
    }
    for name, m in parts.items():
        p = out / f"{name}.stl"
        m.export(p)
        lo, hi = m.bounds
        report["files"][p.name] = {"min_mm": [round(float(v), 2) for v in lo], "max_mm": [round(float(v), 2) for v in hi],
                                   "watertight": bool(m.is_watertight)}
    (out / "parts_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    print(f"{len(parts)} parts -> {out}")
    for fname, info in report["files"].items():
        print(f"  {fname:12s} min {info['min_mm']}  max {info['max_mm']}")
    print("goal_hole:", report["goal_hole"]["center_m"], "r =", report["goal_hole"]["radius_m"], "m")
    print("narrowest wall gap:", report["narrowest_wall_gap_mm"], "mm")
    if problems:
        print("PROBLEMS:\n  " + "\n  ".join(problems))
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()

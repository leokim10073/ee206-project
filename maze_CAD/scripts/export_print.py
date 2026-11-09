"""
UR-aMAZE-ing  —  printable CAD for a maze level.
Outputs STEP (for Autodesk Inventor) and STL (for slicing), one-piece and 4-tile.
Coordinate system: origin = bottom-left corner of base plate (top view),
x right, y up, z up; base bottom at z = 0.  All units mm.

Run:  python3 maze_CAD/scripts/export_print.py --level L1     # -> maze_CAD/levels/L1/print/
Change dimensions in maze_params.py, layouts in levels.py, and re-run.
Needs cadquery (not in the sim container; see the README).
"""
import argparse
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from levels import LEVELS  # noqa: E402
from maze_params import P, wall_rects, cell_center, sticker_size, tag_pads  # noqa: E402

LEVELS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "levels")

# split (for printers smaller than 300x300) — seams chosen to avoid the grip tab
# and to avoid running along any wall
SEAM_X = 150.0
SEAM_Y = 120.0
DOWEL_D = 2.0        # holes for 1.75 mm filament dowels
DOWEL_DEPTH = 8.0    # each side of the seam
DOWEL_PITCH = 40.0


def box(x, y, z, dx, dy, dz):
    return cq.Workplane("XY").box(dx, dy, dz, centered=False).translate((x, y, z))


def build_maze(L):
    B, bt, wh = P["base"], P["base_t"], P["wall_h"]
    body = box(0, 0, 0, B, B, bt)

    # walls
    for (x, y, w, h) in wall_rects(L):
        body = body.union(box(x, y, bt, w, h, wh))

    # grip tab on the left edge, bottom flush with base bottom
    hw, hl, ht, emb = P["handle_w"], P["handle_l"], P["handle_h"], P["tab_embed"]
    tab = box(-hl, B / 2 - hw / 2, 0, hl + emb, hw, ht)
    # round the outer vertical corners of the tab
    tab = tab.edges("|Z and <X").fillet(4)
    body = body.union(tab)

    # finger-pad locating recesses on top and bottom faces of the tab
    px, py, pd = P["pad_x"], P["pad_y"], P["pad_depth"]
    x0 = -hl + 4  # 4 mm lip at the tab tip keeps the fingers from sliding off
    body = body.cut(box(x0, B / 2 - py / 2, ht - pd, px, py, pd + 0.01))
    body = body.cut(box(x0, B / 2 - py / 2, -0.01, px, py, pd + 0.01))

    # ArUco corner pads: top surface flush with maze floor (z = base_t)
    tsz = sticker_size() + 0.5   # +0.5 clearance for the sticker
    for k, (x, y, S, (cx, cy)) in tag_pads().items():
        pad = box(x, y, 0, S, S, bt)
        outer = "<Y" if y < 0 else ">Y"
        pad = pad.edges("|Z").edges(outer).fillet(4)
        body = body.union(pad)
        body = body.cut(box(cx - tsz / 2, cy - tsz / 2, bt - P["tag_recess"], tsz, tsz, P["tag_recess"] + 0.01))

    # goal pocket (with a small chamfer so the ball drops in smoothly)
    gx, gy = cell_center(L, L["goal"])
    pocket = (cq.Workplane("XY").workplane(offset=bt - P["goal_depth"])
              .center(gx, gy).circle(P["goal_d"] / 2).extrude(P["goal_depth"] + 0.01))
    body = body.cut(pocket)
    try:
        body = body.faces(">Z").edges(cq.selectors.NearestToPointSelector((gx + P["goal_d"] / 2, gy, bt))).chamfer(1.0)
    except Exception:
        pass

    # start marker: engraved ring (a ring, not a pocket, so the ball rolls off at a small tilt)
    sx, sy = cell_center(L, L["start"])
    r_in, r_out, ring_d = P["start_ring"]
    ring = (cq.Workplane("XY").workplane(offset=bt - ring_d).center(sx, sy)
            .circle(r_out).circle(r_in).extrude(ring_d + 0.01))
    body = body.cut(ring)
    return body


def dowel_holes(body):
    """horizontal filament-dowel holes crossing both seams, at mid-thickness of the base"""
    B, bt = P["base"], P["base_t"]
    z = bt / 2
    L2 = 2 * DOWEL_DEPTH
    # holes across the vertical seam (x = SEAM_X), along y
    y = 20.0
    while y < B - 10:
        if abs(y - SEAM_Y) > 12:
            h = (cq.Workplane("YZ").workplane(offset=SEAM_X - DOWEL_DEPTH)
                 .center(y, z).circle(DOWEL_D / 2).extrude(L2))
            body = body.cut(h)
        y += DOWEL_PITCH
    # holes across the horizontal seam (y = SEAM_Y), along x
    x = 20.0
    while x < B - 10:
        if abs(x - SEAM_X) > 12:
            h = (cq.Workplane("XZ").workplane(offset=-(SEAM_Y + DOWEL_DEPTH))
                 .center(x, z).circle(DOWEL_D / 2).extrude(L2))
            body = body.cut(h)
        x += DOWEL_PITCH
    return body


def split_tiles(body):
    B = P["base"]
    big = 1000
    regions = {
        "A_top_left":     (-200, SEAM_Y, SEAM_X + 200, big),
        "B_top_right":    (SEAM_X, SEAM_Y, big, big),
        "C_bottom_left":  (-200, -big, SEAM_X + 200, SEAM_Y + big),  # includes bottom ArUco pads
        "D_bottom_right": (SEAM_X, -big, big, SEAM_Y + big),
    }
    tiles = {}
    for name, (x, y, dx, dy) in regions.items():
        cutter = box(x, y, -50, dx, dy, 200)
        tiles[name] = body.intersect(cutter)
    return tiles


def splice_strips():
    """thin strips glued under the seams for extra stiffness (optional)"""
    B = P["base"]; w, t = 30.0, 3.0
    strips = [
        (SEAM_X - w / 2, 8, w, SEAM_Y - 8 - 20),                 # vertical seam, lower
        (SEAM_X - w / 2, SEAM_Y + 20, w, B - 8 - SEAM_Y - 20),   # vertical seam, upper
        (8, SEAM_Y - w / 2, SEAM_X - 8 - 20, w),                 # horizontal seam, left
        (SEAM_X + 20, SEAM_Y - w / 2, B - 8 - SEAM_X - 20, w),   # horizontal seam, right
    ]
    return [box(x, y, -t, dx, dy, t).edges("|Z").fillet(3) for (x, y, dx, dy) in strips]


def bbox_str(wp):
    bb = wp.val().BoundingBox()
    return f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}"


def main():
    ap = argparse.ArgumentParser(description="Printable STEP/STL for a maze level.")
    ap.add_argument("--level", default="L1", choices=sorted(LEVELS))
    a = ap.parse_args()
    L, tag = LEVELS[a.level], a.level
    out = os.path.join(LEVELS_DIR, tag, "print")
    os.makedirs(out, exist_ok=True)
    maze = build_maze(L)

    # 1) one-piece version
    cq.exporters.export(maze, f"{out}/maze_{tag}_onepiece.step")
    cq.exporters.export(maze, f"{out}/maze_{tag}_onepiece.stl", tolerance=0.05, angularTolerance=0.1)
    print("one-piece:", bbox_str(maze), "volume cm3 =", round(maze.val().Volume() / 1000, 1))

    # 2) 4-tile version
    holed = dowel_holes(maze)
    tiles = split_tiles(holed)
    asm = cq.Assembly(name=f"maze_{tag}_tiles")
    for name, t in tiles.items():
        cq.exporters.export(t, f"{out}/maze_{tag}_tile_{name}.stl", tolerance=0.05, angularTolerance=0.1)
        asm.add(t, name=f"tile_{name}")
        print(f"tile {name}:", bbox_str(t))
    for i, s in enumerate(splice_strips()):
        cq.exporters.export(s, f"{out}/maze_{tag}_splice_strip_{i+1}.stl")
        asm.add(s, name=f"splice_strip_{i+1}")
        print(f"strip {i+1}:", bbox_str(s))
    asm.save(f"{out}/maze_{tag}_tiles_assembly.step")
    print("done ->", out)


if __name__ == "__main__":
    main()

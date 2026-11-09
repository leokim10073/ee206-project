"""A stand-in maze board in Steven's export format, used until the real CAD export arrives.

All parts in mm, one frame, origin at the board center, z up, plate top at z = 0:
base.stl (500 x 500 x 6, one 24 mm hole at (100, 100)), two walls, and a handle at the -x edge.
"""

import argparse
from pathlib import Path

import shapely.geometry as sg
import trimesh

PLATE = 500.0
THICKNESS = 6.0
HOLE_CENTER = (100.0, 100.0)
HOLE_RADIUS = 12.0
WALL_HEIGHT = 20.0
WALLS = {  # name: (xmin, xmax, ymin, ymax)
    "wall_0": (-150.0, 150.0, -155.0, -145.0),
    "wall_1": (-5.0, 5.0, 0.0, 200.0),
}
HANDLE_SIZE = (60.0, 40.0, 20.0)
HANDLE_CENTER = (-270.0, 0.0, -3.0)


def _box(xmin, xmax, ymin, ymax, zmin, zmax):
    b = trimesh.creation.box(extents=(xmax - xmin, ymax - ymin, zmax - zmin))
    b.apply_translation(((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2))
    return b


def make_placeholder_board(out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    half = PLATE / 2
    outline = sg.box(-half, -half, half, half).difference(sg.Point(*HOLE_CENTER).buffer(HOLE_RADIUS, 16))
    base = trimesh.creation.extrude_polygon(outline, THICKNESS)
    base.apply_translation((0, 0, -THICKNESS))
    base.export(out_dir / "base.stl")
    for name, (x0, x1, y0, y1) in WALLS.items():
        _box(x0, x1, y0, y1, 0.0, WALL_HEIGHT).export(out_dir / f"{name}.stl")
    handle = trimesh.creation.box(extents=HANDLE_SIZE)
    handle.apply_translation(HANDLE_CENTER)
    handle.export(out_dir / "handle.stl")
    return out_dir


def main():
    p = argparse.ArgumentParser(description="Write the placeholder maze board as STL parts (mm).")
    p.add_argument("out_dir")
    make_placeholder_board(p.parse_args().out_dir)


if __name__ == "__main__":
    main()

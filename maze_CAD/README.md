# maze_CAD

All maze dimensions live in `maze_params.py`; the other scripts read them from there.

| Script | Output | Needs |
|---|---|---|
| `export_parts.py` | `L1_parts/`: separate STL parts for the MuJoCo sim + `parts_report.json` | trimesh, shapely, mapbox_earcut (in the sim container) |
| `maze_cad.py` | `print/`: printable STEP (for Inventor) and STL, one-piece and 4-tile | cadquery |
| `maze_design.py` | `maze_v1_schematic.png` | matplotlib |

## Sim parts (`L1_parts/`)

```bash
python3 maze_CAD/export_parts.py --level L1
python3 -m maze_sim_description.board_assets maze_CAD/L1_parts --units mm \
    --out-dir src/maze_sim_description/meshes/board
```

Frame: origin at the center of the 300×300 base plate, z = 0 at the base bottom, z up, handle on the −x edge, units mm. The maze floor is at z = 6.

- `base.stl`: plate plus the four ArUco pads, with trap holes cut through (level 1 has none)
- `handle.stl`: the grip tab outside the plate
- `wall_<k>.stl`: one straight box per wall, outer frame included; `parts_report.json` maps each k to the wall's name

The goal pocket is not in `base.stl`. Its center and radius are in `parts_report.json` under `goal_hole`, ready for `sim.yaml` `board.goal_hole`.

The script also leaves out print-only details: the start ring, sticker and finger-pad recesses, and the rounded pad corners. It exits non-zero unless every check passes:

- every STL is watertight with outward normals
- every wall's volume matches its convex hull within 1 %
- every gap between walls is wider than 18 mm
- the base's longest side is 200–1000 mm

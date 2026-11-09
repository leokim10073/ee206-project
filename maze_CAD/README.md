# maze_CAD

CAD for the maze boards: one parametric source, three kinds of output per level (print, sim, schematic), plus the ArUco corner stickers shared by every level.

```
maze_CAD/
├── scripts/                 the source: edit these, then regenerate
│   ├── maze_params.py       board dimensions and ArUco ids, the same for every level
│   ├── levels.py            maze layouts (walls, holes, start, goal, path), one dict per level
│   ├── export_print.py      -> levels/<L>/print/   printable STEP + STL (one piece and 4 tiles)
│   ├── export_sim.py        -> levels/<L>/sim/     separate STL parts for the MuJoCo sim
│   ├── draw_schematic.py    -> levels/<L>/schematic.png
│   └── aruco.py             -> aruco/              marker images + printable sticker sheet
├── aruco/                   ArUco stickers, the same four on every board
└── levels/
    └── L1/                  one folder per level, all generated
        ├── schematic.png
        ├── print/
        └── sim/
```

Everything under `levels/` and `aruco/` is generated. Change `scripts/`, rerun, and commit the outputs with the script change.

## Adding a level

1. Add a dict to `scripts/levels.py` (copy `LEVEL1`) and register it in `LEVELS`, e.g. `"L2": LEVEL2`.
2. Run the three exports with `--level L2` (see below). `export_sim.py` exits non-zero if a corridor is too narrow for the ball, or if the goal, start ring or a trap hole hits a wall.
3. Commit `scripts/levels.py` and `levels/L2/`.

## Running the scripts

| Script | Where to run it | Needs |
|---|---|---|
| `export_sim.py` | inside `maze-sim` | trimesh, shapely, mapbox_earcut, opencv (all in the image) |
| `export_print.py` | a Python env with cadquery | cadquery (not in the `maze-sim` image) |
| `draw_schematic.py`, `aruco.py` | either | matplotlib, opencv with `cv2.aruco` |

Inside `maze-sim` (`distrobox enter maze-sim`, from the repo root):

```bash
python3 maze_CAD/scripts/export_sim.py --level L1
```

For the print files, a separate venv works (cadquery is large, so it stays out of the sim image):

```bash
uv venv ~/.venvs/maze-cad --python 3.12
VIRTUAL_ENV=~/.venvs/maze-cad uv pip install cadquery trimesh shapely mapbox_earcut matplotlib opencv-contrib-python-headless
~/.venvs/maze-cad/bin/python maze_CAD/scripts/export_print.py --level L1
~/.venvs/maze-cad/bin/python maze_CAD/scripts/draw_schematic.py --level L1
~/.venvs/maze-cad/bin/python maze_CAD/scripts/aruco.py
```

Use `distrobox enter`, not `podman exec`: files written through `podman exec` belong to a container uid, and your user cannot delete them afterwards.

## Sim parts (`levels/<L>/sim/`)

Frame: origin at the center of the 300×300 plate, **z = 0 on the maze floor** (the plate's top face), z up, handle on the −x edge, units mm. This is the frame the sim's placeholder board uses; the sim places the ball at z = ball radius.

| File | What it is |
|---|---|
| `base.stl` | Plate and the four ArUco pads, with the start ring (0.6 mm), any trap holes, and a through hole for the goal |
| `handle.stl` | The grip tab outside the plate; bottom flush with the base bottom, as printed |
| `wall_<k>.stl` | One straight box per wall, outer frame included. **The last one is the goal pocket's floor**, a Ø30 × 3 mm disk under the goal hole. `parts_report.json` → `wall_names` says which is which |
| `aruco/aruco_<corner>.obj` + `.png` | One textured sticker per corner marker (see below) |
| `parts_report.json` | Bounding boxes, goal and start for `sim.yaml`, handle, ArUco poses, and the check results |

Convert for the sim (inside `maze-sim`, in the sim workspace):

```bash
python3 -m maze_sim_description.board_assets <repo>/maze_CAD/levels/L1/sim --units mm \
    --out-dir src/maze_sim_description/meshes/board
colcon build
```

`sim.yaml` values for each level come from `parts_report.json`: `goal_hole.center_m` and `radius_m` → `board.goal_hole`, `ball_start_m` → `ball.start`.

### Why the goal is a hole plus a disk

The sim splits `base.stl` into convex pieces with CoACD, and CoACD fills a 3 mm pocket back in (it is shallow next to a 416 mm plate), so the ball would roll over the goal. It does keep a through hole open, and it takes every `wall_*.stl` as its exact convex hull. So the base gets a Ø30 through hole and the floor disk closes it 3 mm down. Checked on L1 with the sim's own converter: the collision floor is at −3 mm across the hole out to a radius of 12–13 mm (CoACD rounds the hole slightly smaller than 15 mm), which is enough for the 7 mm sim ball to sit in it.

The start is a 0.6 mm engraved ring, not a pocket: a ball of radius r in a pocket of depth d needs a tilt of acos((r − d)/r) to climb out, so even a 1.5 mm pocket would hold a Ø20 ball until 32°, far beyond the board's ±5°. The ring is filled in the collision geometry (too shallow for CoACD), so it is visual only.

### ArUco markers in the sim

The sim's `board_assets` reads only `base`, `handle` and `wall_*`, and draws every board mesh in one color, so the markers need their own geoms in the scene. Each `aruco_<corner>.obj` is a 50×50×0.1 mm slab (closed, so MuJoCo accepts it) whose top face carries `aruco_<corner>.png` through UV coordinates; it sits on the pad, with the image's top edge toward board +y. In MJCF, inside the board body (paths relative to the sim parts folder, scaled from mm):

```xml
<mesh name="aruco_TL" file="aruco/aruco_TL.obj" scale="0.001 0.001 0.001"/>
<texture name="aruco_TL" type="2d" file="aruco/aruco_TL.png"/>
<material name="aruco_TL" texture="aruco_TL"/>
<geom type="mesh" mesh="aruco_TL" material="aruco_TL" contype="0" conaffinity="0" group="1"/>
```

Checked by rendering the L1 board from above in MuJoCo: OpenCV 5's `ArucoDetector` finds all four ids with the right orientation, down to a 58 px marker. The older detector in the `maze-sim` image (OpenCV 4.5) missed one of the four on the same image.

## Print (`levels/<L>/print/`)

Frame: origin at the plate's bottom-left corner (top view), base bottom at z = 0, units mm. `maze_<L>_onepiece.step/.stl` is the whole 350×416 board; the four `tile_*` pieces (largest 200×238) and `splice_strip_*` are for printers smaller than that, joined with 1.75 mm filament dowels. The print has the goal pocket, the start ring, finger-pad recesses on the tab, and 0.4 mm recesses for the stickers.

## ArUco stickers (`aruco/`)

`aruco_stickers_A4.pdf` prints the four stickers at 1:1 (40 mm marker + 5 mm white border = 50 mm). Print at 100 %, check the 50 mm scale bar, cut on the dashed lines, and stick each one in its corner recess with the labelled edge toward the board's bottom edge. Dictionary and ids (`DICT_4X4_50`, TL 0, TR 1, BR 2, BL 3) are in `scripts/maze_params.py` → `ARUCO`; **confirm them with perception before printing**, then rerun `aruco.py` and `export_sim.py` if they change.

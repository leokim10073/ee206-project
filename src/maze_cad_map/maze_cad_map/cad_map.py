"""The maze map of a CAD level, in the shape of contract 1 (docs/interfaces.md). Plain Python and
numpy, no ROS, so planning and perception tests can use it directly.

Source: maze_CAD/scripts/levels.py (layout) and maze_params.py (dimensions), the files every CAD
export is generated from. Their "plate frame" has its origin at the plate's bottom-left corner, mm,
grid rows counted from the TOP; this module converts to maze_board: origin at the plate center, m,
occupancy-grid rows counted from the BOTTOM (-y) edge.
"""

import importlib.util
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

FREE, WALL, UNKNOWN = 0, 100, -1
DEFAULT_RESOLUTION = 0.005  # m per cell: a 6 mm wall always covers at least one cell center


@dataclass(frozen=True)
class Circle:
    x: float  # m, maze_board
    y: float
    radius: float


@dataclass
class CadMap:
    """Contract 1 without ROS types. grid[row, col], row 0 = the -y edge."""
    source: str
    resolution: float
    origin: tuple  # (x, y) of the grid's lower-left corner, m
    grid: np.ndarray  # int8, FREE or WALL
    holes: list = field(default_factory=list)
    start: tuple = (0.0, 0.0)  # (x, y), m
    goal: Circle = None

    @property
    def width(self):
        return self.grid.shape[1]

    @property
    def height(self):
        return self.grid.shape[0]

    def cell_center(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (row + 0.5) * self.resolution)

    def cell_of(self, x, y):
        """(row, col) of the cell holding board point (x, y)"""
        return (int(np.floor((y - self.origin[1]) / self.resolution)),
                int(np.floor((x - self.origin[0]) / self.resolution)))

    def occupancy_data(self):
        """OccupancyGrid.data order: row-major from row 0 (the -y edge)"""
        return self.grid.astype(np.int8).ravel().tolist()


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or not Path(path).is_file():
        raise FileNotFoundError(f"{path} not found (pass the repo's maze_CAD/scripts directory)")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_cad(cad_scripts):
    """(maze_params, levels) modules from a maze_CAD/scripts directory"""
    cad_scripts = Path(cad_scripts)
    return _load(cad_scripts / "maze_params.py", "maze_params"), _load(cad_scripts / "levels.py", "levels")


def build_map(cad_scripts, level="L1", resolution=DEFAULT_RESOLUTION):
    params, levels = load_cad(cad_scripts)
    if level not in levels.LEVELS:
        raise KeyError(f"level {level!r} not in levels.py (have {sorted(levels.LEVELS)})")
    L = levels.LEVELS[level]
    P = params.P
    base = P["base"]  # mm, square plate
    half = base / 2.0

    def to_board(x_mm, y_mm):
        return ((x_mm - half) / 1000.0, (y_mm - half) / 1000.0)

    n = int(round(base / 1000.0 / resolution))
    if not np.isclose(n * resolution, base / 1000.0):
        raise ValueError(f"resolution {resolution} m does not divide the {base:g} mm plate")
    origin = (-half / 1000.0, -half / 1000.0)

    # cell centers in plate-frame mm; a cell is a wall if its center lies in a wall rectangle
    centers = (np.arange(n) + 0.5) * resolution * 1000.0
    cx, cy = np.meshgrid(centers, centers)  # cy grows with the row index: row 0 = bottom
    wall = np.zeros((n, n), bool)
    for x, y, w, h in params.wall_rects(L):
        wall |= (cx >= x) & (cx <= x + w) & (cy >= y) & (cy <= y + h)
    # the lip outside the outer walls is not maze floor the ball can reach
    m = P["margin"]
    wall |= (cx < m) | (cx > base - m) | (cy < m) | (cy > base - m)
    grid = np.where(wall, WALL, FREE).astype(np.int8)

    holes = []
    for r, c, d in L.get("holes", []):
        hx, hy = to_board(*params.cell_center(L, (r, c)))
        holes.append(Circle(hx, hy, d / 2000.0))
    start = to_board(*params.cell_center(L, L["start"]))
    gx, gy = to_board(*params.cell_center(L, L["goal"]))
    goal = Circle(gx, gy, P["goal_d"] / 2000.0)

    return CadMap(source=f"cad:{level}", resolution=resolution, origin=origin, grid=grid,
                  holes=holes, start=start, goal=goal)


def path_cells_board(cad_scripts, level="L1"):
    """The level's intended route (levels.py `path`) as maze-cell centers in maze_board, m"""
    params, levels = load_cad(cad_scripts)
    L = levels.LEVELS[level]
    half = params.P["base"] / 2.0
    return [((x - half) / 1000.0, (y - half) / 1000.0) for x, y in (params.cell_center(L, rc) for rc in L["path"])]

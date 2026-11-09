"""
Maze level layouts. Add a level by adding a dict here and an entry in LEVELS;
every export script takes --level <key>. Board dimensions shared by all levels are in
maze_params.py.

Grid: n x n cells of size `cell` (mm). Rows are counted from the TOP, columns from the LEFT.
  h_walls: (grid line j counted from the top, first col, last col)
  v_walls: (grid line i counted from the left, first row, last row)
  holes:   (row, col, diameter) trap holes cut through the base
  start, goal: (row, col)
  path:    cells the intended route passes through, start to goal (drawn on the schematic)
"""

LEVEL1 = dict(
    name="Level 1 - Easy",
    n=6, cell=42.0,
    h_walls=[(2, 0, 4), (4, 1, 5)],
    v_walls=[(3, 0, 0), (3, 3, 3), (3, 4, 4)],
    holes=[],
    start=(0, 0), goal=(4, 5),
    path=[(0, 0), (0, 1), (1, 2), (1, 3), (0, 4), (1, 5), (2, 5), (2, 4), (2, 3), (2, 2),
          (2, 1), (3, 0), (4, 0), (5, 1), (5, 2), (5, 3), (5, 4), (4, 5)],
)

LEVELS = {"L1": LEVEL1}

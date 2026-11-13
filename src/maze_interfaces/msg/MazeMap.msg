# Contract 1 (docs/interfaces.md): the maze, once per maze, on /maze/map.
# Published by the CAD map tool (maze_cad_map) until perception P2 replaces it.

std_msgs/Header header        # frame_id = "maze_board"; stamp = capture time of the image the map
                              # was built from (CAD tool: the time it was published)
string source                 # where it came from: "cad:L1", "perception", ...

nav_msgs/OccupancyGrid walls  # 0 = free floor, 100 = wall or outside the outer walls, -1 = unknown
                              # (the planner treats -1 as wall). Holes are NOT in the grid.
                              # Default: resolution 0.005 m, 60 x 60 cells over the 300 x 300 mm plate,
                              # info.origin = the grid's lower-left corner (-0.15, -0.15, 0) in maze_board.
                              # Row 0 = the -y (bottom) edge; cell (row, col) = data[row * width + col];
                              # cell center x = origin.x + (col + 0.5) * resolution, y likewise with row.
                              # walls.header.frame_id = "maze_board" too (so RViz can draw it).

Circle[] holes                # trap holes the ball must not enter (L1: none). Not the goal.
geometry_msgs/Point start     # where the ball starts, z = 0, m
Circle goal                   # goal pocket: the run succeeds when the ball drops in

# Contract 3 (docs/interfaces.md): the planner's route, on every plan or replan, on /maze/route.

uint8 STATUS_OK = 0
uint8 STATUS_NO_PATH = 1           # no route exists: path and corridors are empty

std_msgs/Header header             # frame_id = "maze_board"; stamp = when it was planned
uint32 plan_id                     # +1 on every new plan; a controller resets its progress when it changes
uint8 status

nav_msgs/Path path                 # ball-center waypoints from the ball's position to the goal, z = 0;
                                   # path.header.frame_id = "maze_board"; pose orientations unused
float64[] arc_length               # distance along the path from its first pose, m; one per path pose
Corridor[] corridors               # in path order; consecutive corridors overlap
uint32[] pose_corridor             # index into corridors for each path pose

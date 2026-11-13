# Interfaces v0

This file is the contract between our own modules: what each one publishes, in which frame and units, how often, and what happens when something goes wrong. The message definitions will live in `src/maze_interfaces/msg/` (I2, #28); this file explains them.

Robot-side topics (`/joint_states`, `/forward_velocity_controller/commands`, `/camera/image_raw`, ...) are fixed by the lab driver and copied by the sim; see the README's [Simulation → Interfaces](../README.md#interfaces) table. Perception's board pose topics are in the README's [Perception](../README.md#perception) section.

- Last updated: 2026-11-12
- Status: draft for review (work package [#26](https://github.com/leokim10073/ee206-project/issues/26), I1 [#27](https://github.com/leokim10073/ee206-project/issues/27))
- Owner: Jing Xu
- Reviewers: Zhuoren (perception), Steven (ball control), Chelsea (arm control), Leo (sim)

## Shared rules

- **Board frame** `maze_board`**.** Origin at the plate center, z up, z = 0 on the maze floor, handle on the −x edge, metres. Same as the sim, the CAD (`maze_CAD/README.md`) and perception's TF frame. Every message between our modules uses it.
- **Units.** SI only: m, m/s, rad, rad/s, s. The CAD is in mm; convert once, where it is read.
- **Time.** Every message has a `std_msgs/Header`. In the sim every node runs with `use_sim_time:=true`.
- **No magic numbers.** Ball radius, board size and tilt limits come from config or from a message, never from constants in code.
- **Messages** live in the `maze_interfaces` package. Standard ROS types are used inside them (`nav_msgs/OccupancyGrid`, `nav_msgs/Path`, `geometry_msgs/Point`) so RViz and ROS tools work.
- **Topic names** follow the ones ball control (#11) and arm control (#25) already use: `/maze/`* for the maze and route, `/ball/*` for the ball, `/board/*` for the board tilt.
- **One publisher per topic.** For example, the CAD map tool and perception both publish `/maze/map`; never run both.

## Overview

| # | Contract | Topic | Type | Publisher → Subscribers | Rate | QoS |
|---|---|---|---|---|---|---|
| 1 | Maze map | `/maze/map` | `maze_interfaces/MazeMap` | CAD map tool (now) / perception P2 → planner | Once per maze | reliable, transient local |
| 2 | Ball state | `/ball/state` | `maze_interfaces/BallState` | perception P3/P4 → planner, ball control | Every camera frame, ~60 Hz | best effort, last 1 |
| 3 | Planned route | `/maze/route` (+ `/maze/path`) | `maze_interfaces/PlannedRoute` (+ `nav_msgs/Path`) | planner → ball control | On each plan or replan | reliable, transient local |
| 4a | Tilt command | `/board/tilt_cmd` | `maze_interfaces/BoardTilt` | ball control → arm control | Control rate, ~60 Hz | reliable, last 1 |
| 4b | Measured tilt | `/board/tilt_state` | `maze_interfaces/BoardTilt` | arm control → ball control, system ID | ≥ 60 Hz | best effort, last 1 |

```
 camera ──► perception ──(1) /maze/map──► planner ──(3) /maze/route──► ball control ──(4a) /board/tilt_cmd──► arm control ──► UR7e / sim
               │                            ▲                            ▲    ▲                                   │
               └──────────(2) /ball/state───┴────────────────────────────┘    └──────(4b) /board/tilt_state───────┘
   (CAD map tool publishes (1) until perception P2 exists)
```

"Latched" below means reliable + transient local durability: a node that starts later (or restarts) still receives the last message, like a notice pinned on a board.

---

## Contract 1: Maze map

**Publisher:** the CAD map tool (`maze_cad_map`, interfaces I3) until perception P2 (Zhuoren, #23) is ready. Both publish the same message on the same topic.

**Subscribers:** planner.

**Topic:** `/maze/map` (the CAD tool also publishes the walls alone on `/maze/map/walls` as a `nav_msgs/OccupancyGrid`, for RViz).

**Type:** `maze_interfaces/MazeMap`

```
# MazeMap.msg
std_msgs/Header header        # frame_id = "maze_board"; stamp = capture time of the image the map
                              # was built from (CAD tool: the time it was published)
string source                 # "cad:L1", "perception", ...
nav_msgs/OccupancyGrid walls  # 0 = free floor, 100 = wall or outside the outer walls, -1 = unknown
                              # (the planner treats -1 as wall). Holes are NOT in the grid.
                              # resolution 0.005 m; 60 x 60 cells cover the 300 x 300 mm plate
                              # info.origin = lower-left corner (-0.15, -0.15, 0) in maze_board
                              # row 0 = bottom (-y) edge; cell (row, col) = data[row * width + col]
                              # cell center: x = -0.15 + (col + 0.5) * 0.005, same for y with row
Circle[] holes                # trap holes the ball must avoid (L1: none). Not the goal.
geometry_msgs/Point start     # ball start, z = 0, m
Circle goal                   # goal pocket

# Circle.msg
float64 x        # center in maze_board, m
float64 y
float64 radius   # m
```

**Why a grid for walls and circles for holes:** A* runs directly on a grid, RViz draws it, and a camera naturally produces one. Holes and the goal are round, so a circle is exact, and the planner chooses its own clearance from them. The grid's rows count from the bottom, as in every ROS tool; the CAD counts rows from the top, so the CAD tool flips them.

**Rate:** once per maze, after the map is built. Published again only if the map changes (the planner then replans).

**QoS:** latched (reliable, keep last 1, transient local). There is only one message, so it must not be lost.

**Edge cases:**

- Board not found, or start/goal not found: publish no map and log an error. Never publish a partial map.
- Part of the board hidden (e.g. by the gripper): those cells are −1 (unknown); the planner treats unknown as wall.
- No map received yet: the planner does not plan; ball control keeps the board level.

**How to check:**

- `maze_cad_map`'s tests: the CAD map's walls, start and goal match the sim export (`parts_report.json`) and `sim.yaml`.
- In RViz, `/maze/map/walls` drawn over the sim board: walls line up within one cell (5 mm).
- Perception's L1 map vs the CAD map: they differ only in cells next to a wall; start and goal within 5 mm.

**Example message (L1 from the CAD tool):**

```yaml
header: {stamp: {sec: 3, nanosec: 0}, frame_id: maze_board}
source: cad:L1
walls:
  header: {frame_id: maze_board}
  info: {resolution: 0.005, width: 60, height: 60,
         origin: {position: {x: -0.15, y: -0.15, z: 0.0}, orientation: {w: 1.0}}}
  data: [100, 100, 100, ..., 100, 0, 0, ...]   # 3600 values, row 0 (bottom edge) first
holes: []
start: {x: -0.12, y: 0.12, z: 0.0}
goal: {x: 0.12, y: -0.072, radius: 0.015}
```

**TODO (Zhuoren):** is the goal detected in the image or given from config (#9)?

---

## Contract 2: Ball state

**Publisher:** ball tracker in `maze_perception` (Zhuoren, #23 P3/P4). Until it exists, ball control's stand-in turns `/sim/ball/pose` + `/sim/board/pose` into this message (#11).

**Subscribers:** planner (to check the ball is still in its corridor), ball controllers (PID, LQR, MPC).

**Topic:** `/ball/state`

**Type:** `maze_interfaces/BallState`

```
# BallState.msg
std_msgs/Header header   # stamp = capture time of the camera image this came from (not publish time)
                         # frame_id = "maze_board"
float64 x                # ball center projected on the floor plane (z = 0), m
float64 y                # m
float64 vx               # m/s, from the Kalman filter
float64 vy               # m/s
float64[4] position_covariance   # 2x2 [xx, xy, yx, yy], m^2; all zeros if unknown
bool tracking_ok         # false = ball not detected recently; x, y, vx, vy are the filter's prediction
```

**Rate:** one message per camera frame. About 60 Hz on the robot, 56–62.5 Hz in the sim.

**Timing:** `header.stamp` is the image capture time, so a controller can see how old the state is and predict forward to compensate camera delay (proposal: "Kalman filter estimates velocity and compensates for delay").

**QoS:** best effort, keep last 1. Old ball states are useless; only the newest matters. (`board_node.py` uses the same `SENSOR_QOS`.)

**Edge cases:**

- Ball not detected in a frame: still publish, with the filter's prediction and `tracking_ok = false`.
- `tracking_ok` false for more than 0.2 s: controllers command a level board and the run stops (proposal: "predict briefly, then stop the trial if tracking remains lost").
- Board not found (fewer than 3 ArUco markers): no ball state can be computed; publish nothing. Subscribers treat "no message for 0.1 s" the same as `tracking_ok = false`.

**How to check:** in the sim, convert `/sim/ball/pose` (world frame) into `maze_board` using `/sim/board/pose`; `x, y` must be within a few mm (#23's done-when).

**Example message (L1 start cell, ball at rest):**

```yaml
header: {stamp: {sec: 12, nanosec: 400000000}, frame_id: maze_board}
x: -0.12
y: 0.12
vx: 0.0
vy: 0.0
position_covariance: [1.0e-6, 0.0, 0.0, 1.0e-6]
tracking_ok: true
```

**TODO (Zhuoren):** edit this contract to match the ball tracker; confirm the 0.2 s timeout and who compensates the camera delay.

---

## Contract 3: Planned route

**Publisher:** planner.

**Subscribers:** ball controllers: PID (#11) follows the path; constrained MPC (#15) also uses the corridors as constraints; MPCC (#16) uses the arc length as its progress variable.

**Topic:** `/maze/route`. The planner also publishes the same path alone on `/maze/path` as a `nav_msgs/Path`, the topic `pid_node` already reads (#11) and RViz can draw.

**Type:** `maze_interfaces/PlannedRoute`

```
# PlannedRoute.msg
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

# Corridor.msg: a convex safe region for the BALL CENTER, axis-aligned in maze_board, m.
# Wall clearance (ball radius + margin) is already taken off, so a controller can use the bounds
# directly as constraints on the ball state's x and y.
float64 x_min
float64 x_max
float64 y_min
float64 y_max
```

**Why corridors bound the ball center:** the MPC constrains the ball state `(x, y)`, which is the center. If the planner already subtracts the ball radius and a margin from the walls, the controller needs no geometry of its own, and the clearance lives in one place (planner config).

**Rate:** on every plan or replan: once when the map and a ball state are available, then each time the ball leaves its corridor.

**QoS:** latched (reliable, keep last 1, transient local), so a controller that restarts gets the current route.

**Edge cases:**

- No route exists: publish `status = STATUS_NO_PATH` with empty `path` and `corridors`; ball control keeps the board level.
- Ball leaves its corridor: the planner replans from the ball's position and publishes a new route with `plan_id + 1`. A controller seeing a new `plan_id` starts tracking from the new path's start.
- No route received yet: ball control keeps the board level.

**How to check:** on the CAD map of L1, the route passes through the cells of `levels.py`'s `path`; every path pose lies inside its `pose_corridor` corridor; consecutive corridors overlap; in RViz the path on `/maze/path` stays inside the maze.

**Example message (two corridors, three poses):**

```yaml
header: {stamp: {sec: 4, nanosec: 0}, frame_id: maze_board}
plan_id: 1
status: 0
path:
  header: {frame_id: maze_board}
  poses:
    - pose: {position: {x: -0.12, y: 0.12, z: 0.0}}
    - pose: {position: {x: -0.072, y: 0.12, z: 0.0}}
    - pose: {position: {x: -0.072, y: 0.072, z: 0.0}}
arc_length: [0.0, 0.048, 0.096]
corridors:
  - {x_min: -0.13, x_max: -0.062, y_min: 0.11, y_max: 0.13}
  - {x_min: -0.082, x_max: -0.062, y_min: 0.062, y_max: 0.13}
pose_corridor: [0, 0, 1]
```

**TODO (Steven):** confirm `pid_node` reads `/maze/path` as a `nav_msgs/Path`.

**TODO:** set the ball radius for the corridor clearance once the real ball is bought (sim: 7 mm).

---

## Contract 4: Board tilt (command and measurement)

**4a, command.** Publisher: ball controllers (Steven, #10). Subscriber: arm control (Chelsea, #25). Topic `/board/tilt_cmd`.

**4b, measurement.** Publisher: arm control, from forward kinematics of `/joint_states` (#25). Subscribers: ball controllers, system ID logging (#12). Topic `/board/tilt_state`.

**Type (both):** `maze_interfaces/BoardTilt`

```
# BoardTilt.msg
# Defined by the effect on the ball, relative to the board's level home pose:
#   tilt_x > 0 lowers the board's +x edge, so the ball accelerates toward +x:  a_x = 5/7 g sin(tilt_x)
#   tilt_y > 0 lowers the board's +y edge, so the ball accelerates toward +y:  a_y = 5/7 g sin(tilt_y)
# As rotations of maze_board (right-handed): tilt_x = +angle about board +y,
#                                            tilt_y = -angle about board +x.
std_msgs/Header header   # stamp = when the command was computed (cmd) or measured (state);
                         # frame_id = "maze_board"
float64 tilt_x           # rad
float64 tilt_y           # rad
```

**Why the sign is defined by its effect:** with right-hand rotations alone, a positive rotation about +y sends the ball to +x but a positive rotation about +x sends it to −y. Writing it as "the ball goes toward +x when `tilt_x > 0`" makes both axes match the ball model of #12, so a controller can write `tilt_x = k * error_x` with no sign flips. The rotation line is there for arm control.

**Angles, not rates:** ball control thinks in tilt angles; arm control turns the angle error into joint velocities (resolved-rate control, #25). The board tilts about its own center (the remote pivot); the board center should stay put.

**Rate:** one command per control step, about 60 Hz (the camera rate). Measured tilt at least as fast.

**QoS:** command reliable, keep last 1 (only the newest command matters, but it should arrive). Measured tilt best effort, keep last 1.

**Limits and edge cases:**

- Both sides clamp: the controller to its own `max_tilt` and tilt-rate limits, arm control again to its hard limits (safety). Default `max_tilt` = 0.087 rad (5°) until the grip test (#9) confirms it.
- No command for 200 ms: arm control stops the arm (zero joint velocities) and logs a warning, the same timeout as the PID watchdog (#11, #25).
- `tilt_state` comes from forward kinematics, not from `/perception/board_pose`, so it does not wait on the camera.

**How to check:** in the sim, from `/sim/reset`, publish `tilt_x = +0.035` (2°) for 0.5 s: the ball (`/sim/ball/pose` converted to `maze_board`) moves toward +x, and `/sim/board/pose` shows the board center moving less than 5 mm (#25's done-when). Repeat for `tilt_y`.

**Example message:**

```yaml
header: {stamp: {sec: 15, nanosec: 16000000}, frame_id: maze_board}
tilt_x: 0.035
tilt_y: -0.010
```

**TODO (Chelsea):** confirm the 0.087 rad hard limit and the tilt-rate limit.

**TODO (Steven):** confirm the PID uses this sign.

---

## Changelog

- v0 (2026-11-12): first draft. Contracts 1–4, `maze_interfaces` messages, CAD map tool.

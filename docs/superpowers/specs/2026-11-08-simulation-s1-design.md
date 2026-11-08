# Simulation S1: hardware-faithful UR7e maze scene

Date: 2026-11-08
Owner: Leo Chen (work package 2, Simulation)
Branch: `feat/simulation`

## 1. Goal

Build a MuJoCo simulation of the real lab setup that teammates can use instead of the robot, with **no simplified modes**. Code written against the sim must run on the real UR7e without changes, and the other way round.

S1 is done when a node written like the lab 7 starter code (reads `/joint_states`, publishes to `/forward_velocity_controller/commands`, switches controllers with `ros2 control switch_controllers`) runs unmodified against the sim, the gripper holds the maze board from the first frame, and the overhead camera publishes images from a pose set in config.

## 2. Scope

**In S1**

- UR7e arm, Robotiq 2F-85 gripper, Steven's maze board (base plate, walls, holes, handle) and a ball in one MuJoCo scene.
- The gripper holds the board by its handle through contact and friction from the first frame.
- The UR7e ROS 2 interface as the lab robot exposes it: `/joint_states`, forward velocity, forward position and scaled joint trajectory controllers, controller switching, TF.
- `/toggle_gripper` as in lab 5.
- An overhead camera modeled on the Logitech C922, with its pose, resolution, rate and field of view in config.
- One config file (`sim.yaml`) for every adjustable value, one launch file.
- A few sim-only helpers under `/sim/` (ball ground truth, reset).
- A distrobox image that holds the whole environment. It is built first; nothing is installed on the host.

**Not in S1** (later stages)

- Events and scoring: goal reached, ball in a trap hole, wall contact, run timeout.
- Actuator lag, delay and noise fitted from system identification (work package 8).
- Batch experiment runner and metrics (work package 9).
- Anything that is not part of the robot: perception, planning, ball controllers, IK. Those are other work packages and run outside the sim.

## 3. Key decisions

| Decision | Choice | Reason |
|---|---|---|
| Robot interface | Real `ros2_control` with MuJoCo as the hardware, through `mujoco_ros2_control` | The controllers are the same code the robot runs (`ur_controllers`), so names, parameters, switching and trajectory behavior match by construction. A hand-written imitation would be a simplification. |
| Arm model | MuJoCo Menagerie `universal_robots_ur5e` for dynamics and visuals; `ur_description` with `ur_type:=ur7e` for URDF, TF and joint names | Menagerie has no UR7e. The UR7e has the UR5e's 850 mm reach and link layout, and `ur_description` copies the UR5e physical parameters for the UR7e. Verified in the first implementation step (section 9). |
| Gripper model | MuJoCo Menagerie `robotiq_2f85` | Matches the lab gripper; already tuned for MuJoCo. |
| Board grasp | Physical: finger contact and friction on the form-closure handle. No weld constraint. | Matches the real robot: opening the gripper drops the board. |
| Maze geometry | Steven's CAD, exported per part to STL | The CAD is the single source of the maze; changing mazes means replacing STL files. |
| Where it runs | Each member's own Linux machine, inside distrobox | Members can install packages; distrobox gives everyone the same environment. |

## 4. Architecture

```
 teammate code (same as on the robot)
        │  topics / actions / services
        ▼
 ┌──────────────────────────── distrobox: Ubuntu 22.04 + ROS 2 Humble ────────────────────────────┐
 │                                                                                                 │
 │  controller_manager (inside mujoco_ros2_control)                                                │
 │   ├─ joint_state_broadcaster              → /joint_states (6 UR joints)                         │
 │   ├─ scaled_joint_trajectory_controller   ← trajectory topic and action   (active at start)     │
 │   ├─ forward_velocity_controller          ← /forward_velocity_controller/commands (inactive)    │
 │   ├─ forward_position_controller          ← /forward_position_controller/commands (inactive)    │
 │   └─ gripper controller                   ← driven by the gripper node                          │
 │          ↕                                                                                       │
 │  MuJoCo scene: UR7e + Robotiq 2F-85 + maze board + ball + overhead camera                       │
 │          │                                                                                       │
 │  camera output → /camera/image_raw, /camera/camera_info                                         │
 │  sim helpers   → /sim/ball/pose, /sim/reset, /clock                                             │
 │                                                                                                 │
 │  robot_state_publisher (ur_description, ur7e) → /tf, /tf_static                                 │
 │  gripper node: /toggle_gripper (std_srvs/Trigger) → gripper controller                          │
 └─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

The simulation owns the physics. Camera images, ball ground truth and the gripper actuator are produced inside the `mujoco_ros2_control` process, because only that process can read the MuJoCo state.

### Packages

| Package | Contents |
|---|---|
| `maze_sim_description` | MJCF scene (arm, gripper, board, ball, camera), board meshes and their convex decompositions, ros2_control xacro (UR7e description with the MuJoCo hardware plugin), `config/sim.yaml` |
| `maze_sim_bringup` | `launch/sim.launch.py`, `config/controllers.yaml`, gripper node, sim helper node (if not provided by the plugin) |

Repository layout after S1:

```
src/maze_sim_description/
src/maze_sim_bringup/
docker/Containerfile
docker/distrobox.ini
docs/superpowers/specs/2026-11-08-simulation-s1-design.md
README.md            (sim quickstart and interface table added)
```

## 5. Interfaces

Names, message types and behavior match the lab UR7e. Teammates switch between sim and robot by changing the launch file, not their code.

### Arm (provided by ros2_control, same as the robot)

| Name | Type | Notes |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | The 6 UR joints only, as on the robot. 500 Hz. |
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | 6 joint velocities, rad/s |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 6 joint positions, rad |
| `/scaled_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Used in lab 3 |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | action `control_msgs/FollowJointTrajectory` | Used in lab 7 |
| `/controller_manager/*` | `controller_manager_msgs` services | `ros2 control switch_controllers` works unchanged |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `world` → `base_link` → … → `wrist_3_link` → `tool0`, from `ur_description` |

Controller names and parameters are copied from the Universal Robots ROS 2 driver's controller configuration for the UR7e. At start, `scaled_joint_trajectory_controller` is active and the two forward controllers are loaded but inactive, as on the robot.

### Gripper

| Name | Type | Notes |
|---|---|---|
| `/toggle_gripper` | `std_srvs/Trigger` | Each call opens a closed gripper or closes an open one, as in lab 5. The response says which. |

The gripper joint is not published on `/joint_states`, so that topic carries the same 6 joints as on the robot.

### Camera (Logitech C922 model)

| Name | Type | Notes |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `rgb8`, resolution and rate from `sim.yaml` (default 1280×720 at 60 fps) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Pinhole intrinsics computed from resolution and horizontal field of view, zero distortion |
| `/tf_static` | `world` → `camera_optical_frame` | From the camera pose in `sim.yaml`, ROS optical-frame convention (z forward, x right, y down) |

The real camera's topic names depend on the driver the team picks. Either driver output or sim output is remapped in the launch file so that consumers always read `/camera/image_raw` and `/camera/camera_info`.

### Sim only (no robot equivalent; all under `/sim/`)

| Name | Type | Notes |
|---|---|---|
| `/sim/ball/pose` | `geometry_msgs/PoseStamped` | Ball ground truth in `world`, for checking perception |
| `/sim/reset` | `std_srvs/Trigger` | Arm to `initial_joints`, gripper closed on the handle, board in the grasp, ball at `ball.start`, at rest |
| `/clock` | `rosgraph_msgs/Clock` | Sim time. Every node runs with `use_sim_time:=true`. |

## 6. Scene and assets

### World

- `world` origin at the UR base; the base sits on a table plane at z = 0.
- Physics timestep 2 ms (500 Hz, the UR control rate); controller manager update rate 500 Hz.
- Contact settings chosen for a stable grasp and rolling ball: elliptic friction cones, `noslip` iterations, solver parameters in `sim.yaml`.

### Maze board

Steven provides the board as STEP. MuJoCo cannot read STEP, so the board is exported to STL with these rules:

| Part | Export | Use in the sim |
|---|---|---|
| Each wall | One STL per wall | Walls are boxes, so MuJoCo's convex hull equals the wall: exact collision |
| Base plate with holes | One STL | Convex-decomposed with CoACD so the holes stay open; also used as the visual |
| Handle | One STL | Convex-decomposed with CoACD; the fingers contact it |
| Goal hole | Its center and radius, from the CAD | Stored in `sim.yaml` for later event detection (S2) |

Export rules for Steven: one file per part, all parts in one shared frame with the origin at the board center, z up, and units stated (mm or m). Changing the maze means replacing these files and rerunning the decomposition script.

The board surface (felt) friction comes from `sim.yaml`; system identification later replaces the default.

### Ball

Steel sphere. Radius, mass, start position on the board and friction (sliding, torsional, rolling) come from `sim.yaml`.

### Grasp at start

At load and on `/sim/reset`, the arm is placed at `initial_joints`, the fingers are set closed on the handle, and the board pose is computed from `board.grasp_offset` (handle frame relative to `tool0`). From then on only finger contact holds the board. Opening the gripper drops it.

If the grasp slips beyond the test limit (section 10) after contact tuning, the fallback is a weld constraint. It is used only with the owner's approval.

## 7. Configuration: `sim.yaml`

Every adjustable value lives here. Values below are defaults; board dimensions come from Steven's CAD, and the camera pose is later matched to the real camera.

```yaml
camera:
  position: [0.50, 0.00, 1.40]       # world, m
  look_at:  [0.50, 0.00, 0.40]       # point the optical axis passes through, world, m
  up:       [1.0, 0.0, 0.0]          # world direction that appears as image "up"
  resolution: [1280, 720]
  fps: 60
  hfov_deg: 70.42                    # C922 diagonal 78° at 16:9

robot:
  ur_type: ur7e
  initial_joints: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]   # rad; replaced by the real home pose

gripper:
  closed_position: 0.8               # fraction of full close at which fingers meet the handle
  force_limit: 100.0                 # N

board:
  mesh_dir: package://maze_sim_description/meshes/board
  units: mm                          # units of the exported STL files
  grasp_offset:                      # handle frame relative to tool0
    xyz: [0.0, 0.0, 0.15]
    rpy: [0.0, 0.0, 0.0]
  mass: 0.8                          # kg
  surface_friction: [1.0, 0.005, 0.0001]   # sliding, torsional, rolling
  goal_hole: {center: [0.20, 0.20], radius: 0.012}   # board frame, m

ball:
  radius: 0.007                      # m
  mass: 0.011                        # kg
  start: [-0.20, -0.20]              # board frame, m
  friction: [1.0, 0.005, 0.0001]

physics:
  timestep: 0.002
  real_time_factor: 1.0              # 1.0 = wall-clock; larger runs faster if the machine allows
  noslip_iterations: 3
```

The launch file validates the file at start and refuses to start with a message naming the bad key if a value is missing, has the wrong type, or is out of range (for example a camera `look_at` equal to its `position`).

## 8. Launch

```bash
ros2 launch maze_sim_bringup sim.launch.py config:=<path to sim.yaml> viewer:=true
```

| Argument | Default | Meaning |
|---|---|---|
| `config` | the package's `config/sim.yaml` | Scene and camera configuration |
| `viewer` | `true` | Open the MuJoCo viewer window; `false` runs headless |
| `camera` | `true` | Render and publish the overhead camera |

It starts `robot_state_publisher`, `mujoco_ros2_control` with the controller manager, the controller spawners (active and inactive sets as in section 5), the gripper node, the camera static transform and the sim helpers.

## 9. Risks and the first step

**Plugin capability (first implementation step).** Before building the scene, a short check on Humble:

1. `mujoco_ros2_control` builds and runs on ROS 2 Humble with MuJoCo 3.3.0.
2. It can publish an MJCF camera as `sensor_msgs/Image` and `CameraInfo` at the configured rate.
3. It can drive the Robotiq actuator through a ros2_control command interface.
4. A free body's pose (the ball) and a model reset can be exposed to ROS.
5. `ur_description`'s `ur7e` and `ur5e` `default_kinematics.yaml` match. If they differ, the arm MJCF is generated from the UR7e URDF with MuJoCo's URDF importer, and actuator gains and joint damping are copied from the Menagerie UR5e.

For any of items 2–4 the plugin cannot do, the work stays inside ros2_control: either extend the plugin or write a small ros2_control hardware plugin of our own around MuJoCo. The owner chooses after seeing the check's results.

**Grasp stability.** Contact grasps in MuJoCo can creep. Mitigation: form-closure handle, elliptic cones, `noslip` iterations, finger force limit. Fallback in section 6.

**Real-time speed.** Rendering 1280×720 at 60 fps plus 500 Hz physics must keep real-time factor ≥ 1.0 on a member's laptop. The camera renders offscreen on the GPU; if it cannot keep up, the camera rate drops before the physics rate does, and the drop is logged.

## 10. Acceptance tests

| Test | Pass condition |
|---|---|
| Interfaces | Every name in section 5 exists with the listed type |
| Velocity control | A constant velocity on one joint moves it at that rate within 2% after 0.2 s |
| Position control | A position command is reached within 0.01 rad |
| Trajectory control | A `FollowJointTrajectory` goal through three waypoints succeeds |
| Controller switching | `ros2 control switch_controllers` swaps the trajectory and velocity controllers, as in lab 7 |
| Grasp | While the arm moves through a 60 s joint trajectory, the board moves less than 1 mm relative to `tool0` |
| Release | `/toggle_gripper` opens the gripper and the board falls |
| Camera | Image rate within 5% of `fps`; the whole board is in frame at `initial_joints` with the default pose; `camera_info` matches resolution and field of view |
| Ball | Tilting the board rolls the ball, walls stop it, and it falls through a hole |
| Reset | `/sim/reset` restores the start state, and two runs of the same command sequence from reset produce the same ball path |
| Lab compatibility | A node written like the lab 7 starter (`/joint_states` in, `/forward_velocity_controller/commands` out, controller switch) runs unchanged |
| Speed | Real-time factor ≥ 1.0 with the camera on, on the owner's machine |

Interface, control, grasp and reset tests are automated (`launch_testing`). Camera framing and the viewer are checked by hand once.

## 11. Distrobox packaging

Built first, so nothing is installed on the host: every package lives in the image, and all development and tests run inside the container. The host only needs podman and distrobox. Because distrobox shares the host home, the container sets `PYTHONNOUSERSITE=1`, installs Python packages system-wide in the image, and never writes the user's dotfiles.

- `docker/Containerfile`: Ubuntu 22.04 with ROS 2 Humble; apt packages for `ros-humble-ur`, `ros2_control`, `ros2_controllers`, `xacro`; MuJoCo 3.3.0; CoACD; `mujoco_ros2_control` built from source at a pinned commit; this workspace's dependencies installed with `rosdep`.
- `docker/distrobox.ini`: one container named `maze-sim` with `nvidia=true`, created with `distrobox assemble create --file docker/distrobox.ini`.
- Members install `podman` (or Docker) and `distrobox` on their machine. Each member builds the image from the Containerfile. Publishing a prebuilt image to GitHub Container Registry is a separate decision.
- Because distrobox shares the host network, the same container reaches the lab robot without extra DDS setup.
- README gets: create the container, build the workspace, launch the sim, and the interface table from section 5.

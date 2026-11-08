# Simulation S1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A MuJoCo simulation of the lab UR7e holding the maze board, exposing the robot's real ROS 2 interface, so teammates' code runs unchanged on sim and robot.

**Architecture:** `mujoco_ros2_control` (ros-controls) runs MuJoCo as the ros2_control hardware, so the UR controllers are the same code as on the robot. A Python scene builder turns `sim.yaml` plus board meshes into the MJCF the plugin loads. Small Python nodes add `/toggle_gripper` and the `/sim/` helpers.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, `mujoco_ros2_control` + `mujoco_ros2_control_plugins`, `ros-humble-ur` (`ur_description`, `ur_controllers`), `ros2_controllers`, MuJoCo Python bindings, CoACD, trimesh, pytest, `launch_testing`, distrobox + podman.

**Spec:** `docs/superpowers/specs/2026-11-08-simulation-s1-design.md`

## Global Constraints

- Nothing is installed on the host. Every package lives in the `maze-sim` container image (Task 0); all build, run and test commands run inside `distrobox enter maze-sim`. The host already has podman 3.4.4 and distrobox 1.8.2.5.
- ROS 2 Humble on Ubuntu 22.04; every node runs with `use_sim_time:=true`.
- Physics timestep 0.002 s; controller manager `update_rate: 500`.
- `ur_type: ur7e` for `ur_description`; joint names `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.
- Interface names and types exactly as spec section 5. `/joint_states` carries the 6 UR joints only.
- At start `scaled_joint_trajectory_controller` is active; `forward_velocity_controller` and `forward_position_controller` are loaded and inactive.
- The grasp is contact only. No weld or equality constraint between gripper and board without the owner's approval.
- Every adjustable value comes from `sim.yaml`; a bad value stops launch with a message naming the key.
- Sim-only interfaces live under `/sim/`.
- Package names: `maze_sim_description`, `maze_sim_bringup`. Commit message prefix style: `[sim] ...`, `[docs] ...`.

## Review Focus

1. Board STLs exported in mm but `board.units: m` (or the reverse): a 1000× wrong board must be refused with a message about units, not loaded. Pinned in Task 3.
2. Camera `up` vector parallel to the viewing direction: the camera rotation is undefined; validation must refuse it. Pinned in Task 2.
3. A teammate publishes a command array with the wrong length (5 values) to a forward controller: the arm must not move and the sim must keep running. Pinned in Task 6.
4. The machine cannot keep real time: sim time must slow down consistently (no skipped physics) and the measured real-time factor must be logged. Pinned in Task 9.
5. `/sim/reset` after the board was released and fell: the reset must restore the grasp, not leave the board on the floor. Pinned in Task 8.

## File Structure

```
src/maze_sim_description/
  package.xml, setup.py, setup.cfg, resource/maze_sim_description
  maze_sim_description/
    config.py          # load and validate sim.yaml -> SimConfig
    camera_math.py     # look_at -> quaternion, hfov -> intrinsics
    board_assets.py    # STL parts -> scaled, decomposed meshes + MJCF fragment
    scene_builder.py   # SimConfig + board assets -> full MJCF string
  scripts/build_board_assets   # CLI wrapper around board_assets
  scripts/make_placeholder_board  # writes a 50x50 cm test board as STL parts
  config/sim.yaml
  mjcf/menagerie/universal_robots_ur5e/   # vendored, with LICENSE
  mjcf/menagerie/robotiq_2f85/            # vendored, with LICENSE
  meshes/board/                           # generated; placeholder until Steven's export
  urdf/maze_sim.urdf.xacro
  test/test_config.py, test_camera_math.py, test_board_assets.py, test_scene.py, test_physics.py
src/maze_sim_bringup/
  package.xml, setup.py, setup.cfg, resource/maze_sim_bringup
  maze_sim_bringup/
    gripper_node.py    # /toggle_gripper
    sim_tools_node.py  # /sim/ball/pose, /sim/reset, real-time factor log (only what the plugin lacks)
  launch/sim.launch.py
  config/controllers.yaml
  test/test_interfaces.py, test_arm_control.py, test_gripper.py, test_camera.py, test_sim_tools.py
  test/lab7_style_node.py
docker/Containerfile, docker/distrobox.ini, docker/build.sh
README.md
```

---

### Task 0: Distrobox environment

Spec section 11, done first so nothing is installed on the host. The image grows later only by editing the Containerfile and rebuilding.

**Files:**
- Create: `docker/Containerfile`, `docker/distrobox.ini`, `docker/build.sh`
- Modify: `README.md` (container section)

**Interfaces:**
- Produces: image `localhost/maze-sim:latest`; container `maze-sim` created with `distrobox assemble create --file docker/distrobox.ini`; inside it `ros2`, `colcon`, `xacro`, `ur_description` (`ur7e`), `ur_controllers`, `ros2_controllers`, `mujoco_ros2_control`, `mujoco_ros2_control_plugins`, Python `mujoco`, `coacd`, `trimesh`, `pytest` are available

- [ ] **Step 1: Write `docker/Containerfile`**: `FROM docker.io/osrf/ros:humble-desktop`; `apt-get install` `ros-humble-ur ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-mujoco-ros2-control ros-humble-mujoco-ros2-control-plugins ros-humble-launch-testing-ament-cmake python3-pip python3-pytest libegl1 libgl1`; `pip install mujoco coacd trimesh` with versions pinned to what resolves today (record them in the file). If the two `mujoco-ros2-control` apt packages do not exist for Humble, build `https://github.com/ros-controls/mujoco_ros2_control` from source in `/opt/mujoco_ws` at a pinned tag and source it from `/etc/profile.d/`.
- [ ] **Step 2: Write `docker/build.sh`** (`podman build -t maze-sim -f docker/Containerfile docker/`) and `docker/distrobox.ini`:

```ini
[maze-sim]
image=localhost/maze-sim:latest
pull=false
nvidia=true
init_hooks="echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc"
additional_flags="--env PYTHONNOUSERSITE=1"
```

distrobox shares the host home, so the container must not write the owner's dotfiles or `~/.local`: no `pip install --user`, no edits to `~/.bashrc`; ROS is sourced from the image's `/etc/bash.bashrc`. `PYTHONNOUSERSITE=1` stops the host's `~/.local` Python packages (the host has `mujoco` 3.3.0 there) from shadowing the image's.

- [ ] **Step 3: Owner builds and creates** (in a normal terminal; rootless podman may not run inside the agent sandbox):

```bash
./docker/build.sh
distrobox assemble create --file docker/distrobox.ini
```

- [ ] **Step 4: Verify inside the container**

```bash
distrobox enter maze-sim -- bash -lc 'ros2 pkg list | grep -E "^(ur_description|ur_controllers|mujoco_ros2_control|mujoco_ros2_control_plugins)$"; python3 -c "import mujoco, coacd, trimesh; print(mujoco.__version__)"; nvidia-smi -L'
```

Expected: the four package names, a MuJoCo version, and the GPU line.

- [ ] **Step 5: README** container section: prerequisites (podman, distrobox), build, create, enter, where the workspace is (the host home is shared, so the repo path is the same inside).
- [ ] **Step 6: Commit** `[sim] distrobox environment`

---

### Task 1: Plugin capability check (gate)

Spec section 9. Produces a findings note and stops for the owner's decision. No product code.

**Files:**
- Create: `docs/superpowers/notes/2026-11-08-plugin-check.md`
- Create (throwaway, not committed): a scratch workspace with the plugin demos

- [ ] **Step 1: Enter the container** (`distrobox enter maze-sim`). All steps below run there. Record the `mujoco_ros2_control` version or tag Task 0 installed.

- [ ] **Step 2: Check each capability** and record pass/fail with evidence (command and output) in the note:
  1. The plugin's demo launches on Humble; record the MuJoCo version it links against (the repo states testing with 3.4.0; the pip bindings here are 3.3.0).
  2. `CameraPlugin` publishes `sensor_msgs/Image` and `CameraInfo` for an MJCF camera; measure the rate at 1280×720 with `ros2 topic hz` and the plugin's own log.
  3. An MJCF position actuator (the Robotiq tendon actuator) can be exposed as a ros2_control command interface.
  4. A free body's pose can be read out (plugin, sensor interface, or a `framepos`/`framequat` sensor exposed as a state interface), and the model can be reset to a keyframe from ROS.
  5. Compare `default_kinematics.yaml` and `physical_parameters.yaml` of `ur5e` and `ur7e` in `/opt/ros/humble/share/ur_description/config/`: `diff` output.
  6. Whether the plugin publishes `/clock`.

- [ ] **Step 3: Write the note** with a table (capability, result, evidence) and, for each failed item, the two options from spec section 9 (extend the plugin, or own hardware plugin) with an effort estimate.

- [ ] **Step 4: Commit the note and stop**

```bash
git add docs/superpowers/notes/2026-11-08-plugin-check.md
git commit -m "[docs] mujoco_ros2_control capability check on Humble"
```

Report the table to the owner. Tasks 5, 7 and 8 depend on these results; do not start them until the owner has chosen how to cover any failed item. Tasks 2–4 do not depend on it and may proceed.

---

### Task 2: Config loading and camera math

**Files:**
- Create: `src/maze_sim_description/{package.xml,setup.py,setup.cfg,resource/maze_sim_description}`, `maze_sim_description/__init__.py`, `config.py`, `camera_math.py`, `config/sim.yaml`
- Test: `test/test_config.py`, `test/test_camera_math.py`

**Interfaces:**
- Produces:
  - `class ConfigError(ValueError)` whose message starts with the dotted key, e.g. `"camera.look_at: ..."`
  - `load_sim_config(path: str | Path) -> SimConfig` (frozen dataclasses `SimConfig`, `CameraConfig`, `RobotConfig`, `GripperConfig`, `BoardConfig`, `BallConfig`, `PhysicsConfig`; fields and defaults exactly as spec section 7)
  - `look_at_quaternion(position, look_at, up) -> tuple[float, float, float, float]` (w, x, y, z) of the ROS optical frame (z forward, x right, y down) in `world`
  - `mujoco_camera_quaternion(position, look_at, up) -> tuple[float,float,float,float]` for the MJCF camera (MuJoCo cameras look along −z with y up)
  - `pinhole_intrinsics(width: int, height: int, hfov_deg: float) -> tuple[float, float, float, float]` returning `(fx, fy, cx, cy)` with square pixels

- [ ] **Step 1: Write failing tests**

```python
def test_default_file_loads():
    cfg = load_sim_config(PKG / "config/sim.yaml")
    assert cfg.physics.timestep == 0.002
    assert cfg.camera.resolution == (1280, 720) and cfg.camera.fps == 60
    assert cfg.robot.ur_type == "ur7e" and len(cfg.robot.initial_joints) == 6

@pytest.mark.parametrize("mutate,key", [
    (lambda d: d["camera"].pop("position"), "camera.position"),
    (lambda d: d["camera"].update(fps="sixty"), "camera.fps"),
    (lambda d: d["camera"].update(look_at=d["camera"]["position"]), "camera.look_at"),
    (lambda d: d["camera"].update(up=[0, 0, 1], position=[0, 0, 2], look_at=[0, 0, 0]), "camera.up"),
    (lambda d: d["robot"].update(initial_joints=[0.0] * 5), "robot.initial_joints"),
    (lambda d: d["board"].update(units="inch"), "board.units"),
    (lambda d: d["ball"].update(radius=-0.01), "ball.radius"),
    (lambda d: d["physics"].update(timestep=0), "physics.timestep"),
])
def test_bad_values_name_the_key(tmp_path, mutate, key):
    d = yaml.safe_load(DEFAULT.read_text()); mutate(d)
    p = tmp_path / "sim.yaml"; p.write_text(yaml.safe_dump(d))
    with pytest.raises(ConfigError, match=f"^{re.escape(key)}"):
        load_sim_config(p)

def test_unknown_key_rejected(tmp_path):  # catches typos like "postion"
    d = yaml.safe_load(DEFAULT.read_text()); d["camera"]["postion"] = [0, 0, 1]
    p = tmp_path / "sim.yaml"; p.write_text(yaml.safe_dump(d))
    with pytest.raises(ConfigError, match=r"^camera\.postion"):
        load_sim_config(p)
```

```python
def test_intrinsics_c922():
    fx, fy, cx, cy = pinhole_intrinsics(1280, 720, 70.42)
    assert fx == pytest.approx(640 / math.tan(math.radians(35.21)), rel=1e-3)
    assert fx == fy and (cx, cy) == (639.5, 359.5)

def test_optical_frame_looks_down():
    q = look_at_quaternion((0.5, 0, 1.4), (0.5, 0, 0.4), (1, 0, 0))
    R = quat_to_matrix(q)
    assert np.allclose(R[:, 2], [0, 0, -1])   # optical z points at the board
    assert np.allclose(R[:, 1], [-1, 0, 0])   # image down = -up

def test_mujoco_and_optical_agree():
    args = ((0.3, 0.2, 1.2), (0.5, 0, 0.4), (1, 0, 0))
    Ro = quat_to_matrix(look_at_quaternion(*args)); Rm = quat_to_matrix(mujoco_camera_quaternion(*args))
    assert np.allclose(-Rm[:, 2], Ro[:, 2])   # MuJoCo looks along -z, optical along +z
    assert np.allclose(Rm[:, 1], -Ro[:, 1])   # MuJoCo y up, optical y down
    assert np.allclose(Rm[:, 0], Ro[:, 0])    # both x right
```

- [ ] **Step 2: Run** `cd src/maze_sim_description && python3 -m pytest test/test_config.py test/test_camera_math.py -v`. Expected: FAIL (module not found).

- [ ] **Step 3: Implement** `config.py` (PyYAML, explicit per-field checks; positive numbers for radii, masses, timestep, fps; `units` in `{"mm","m"}`; `up` not parallel to `look_at - position` within 1e-6) and `camera_math.py` (numpy only). Write `config/sim.yaml` with the spec section 7 defaults.

- [ ] **Step 4: Run tests.** Expected: PASS.

- [ ] **Step 5: Commit** `[sim] sim.yaml loader and camera math`

---

### Task 3: Board asset pipeline

**Files:**
- Create: `maze_sim_description/board_assets.py`, `scripts/build_board_assets`, `scripts/make_placeholder_board`, `meshes/board/` (generated placeholder, committed)
- Test: `test/test_board_assets.py`, `test/conftest.py` (fixture `placeholder_mm` = a temp dir filled by `make_placeholder_board`; helper `wrap_minimal_model(fragment: str, meshdir: Path) -> str` that wraps a fragment in `<mujoco><compiler meshdir=.../>…</mujoco>`)

**Interfaces:**
- Consumes: `BoardConfig` (Task 2)
- Produces:
  - Input convention: a directory with `base.stl`, `handle.stl`, `wall_*.stl`, all in one frame, origin at board center, z up (spec section 6).
  - `build_board_assets(src_dir: Path, out_dir: Path, units: str) -> BoardAssets` where `BoardAssets` has `visual_meshes: list[Path]`, `collision_meshes: list[Path]`, `extent_m: tuple[float, float, float]`, `handle_center_m: tuple[float, float, float]`
  - `board_mjcf_fragment(assets: BoardAssets, mass: float, friction: tuple[float, float, float]) -> str`: `<asset>` mesh entries and a `<body name="board">` with one visual geom per part and one collision geom per convex piece
  - CLI: `ros2 run maze_sim_description build_board_assets <src_dir> --units mm` writes into `meshes/board/`

- [ ] **Step 1: Write failing tests** using `make_placeholder_board(dir)` output (500×500×6 mm base with one 24 mm hole at (100, 100) mm, two walls, a 60×40×20 mm handle at the −x edge, exported in mm):

```python
def test_scales_mm_to_m(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path, units="mm")
    assert a.extent_m[0] == pytest.approx(0.5, abs=1e-3)

def test_wrong_units_refused(placeholder_mm, tmp_path):
    with pytest.raises(ValueError, match="units"):
        build_board_assets(placeholder_mm, tmp_path, units="m")   # 500 m board

def test_walls_are_single_convex_pieces(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path, units="mm")
    wall_pieces = [p for p in a.collision_meshes if p.name.startswith("wall_")]
    assert len(wall_pieces) == 2                      # one piece per wall file

def test_hole_stays_open(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path, units="mm")
    base = [trimesh.load(p) for p in a.collision_meshes if p.name.startswith("base_")]
    ray = dict(ray_origins=[[0.100, 0.100, 1.0]], ray_directions=[[0, 0, -1]])
    assert not any(m.ray.intersects_any(**ray)[0] for m in base)   # hole center is open
    ray["ray_origins"] = [[-0.100, -0.100, 1.0]]
    assert any(m.ray.intersects_any(**ray)[0] for m in base)       # solid plate elsewhere

def test_fragment_compiles(placeholder_mm, tmp_path):
    a = build_board_assets(placeholder_mm, tmp_path, units="mm")
    frag = board_mjcf_fragment(a, mass=0.8, friction=(1.0, 0.005, 0.0001))
    m = mujoco.MjModel.from_xml_string(wrap_minimal_model(frag, meshdir=tmp_path))
    assert m.body("board").mass[0] == pytest.approx(0.8)
```

Extent check rule: refuse if the largest extent after scaling is outside 0.2–1.0 m, with a message that names `board.units`.

- [ ] **Step 2: Run** `python3 -m pytest test/test_board_assets.py -v`. Expected: FAIL.
- [ ] **Step 3: Implement** with `trimesh` for loading and scaling and `coacd` for `base.stl` and `handle.stl` (threshold 0.02); walls are passed through as single convex hulls. Write OBJ files. Generate the committed placeholder into `meshes/board/`.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** `[sim] board mesh pipeline and placeholder board`

---

### Task 4: MJCF scene and pure-MuJoCo physics

The whole scene compiles and behaves correctly in MuJoCo alone, before ROS is involved.

**Files:**
- Create: `maze_sim_description/scene_builder.py`, `mjcf/menagerie/universal_robots_ur5e/`, `mjcf/menagerie/robotiq_2f85/` (copied from `google-deepmind/mujoco_menagerie` at a pinned commit, LICENSE files kept, commit hash in a `VENDORED.md`)
- Test: `test/test_scene.py`, `test/test_physics.py`

**Interfaces:**
- Consumes: `SimConfig`, `mujoco_camera_quaternion` (Task 2); `BoardAssets`, `board_mjcf_fragment` (Task 3)
- Produces:
  - `build_scene(cfg: SimConfig, board_dir: Path) -> str` (MJCF text) and `write_scene(cfg, board_dir, out_path: Path) -> Path`
  - Names later tasks rely on: joints as in Global Constraints; gripper actuator `fingers_actuator` (Menagerie name); bodies `board`, `ball` (free joint `ball_free`); camera `overhead_camera`; site `tool0` on the arm flange; keyframe `start`
  - Keyframe `start`: arm at `initial_joints`, fingers at `gripper.closed_position`, board pose = `tool0` pose × `grasp_offset`⁻¹ of the handle frame, ball at rest on the board surface at `ball.start`

- [ ] **Step 1: Write failing tests**

```python
def test_compiles_with_names(model):
    for j in UR_JOINTS: model.joint(j)
    model.camera("overhead_camera"); model.body("board"); model.body("ball")
    assert model.opt.timestep == 0.002
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.noslip_iterations == 3
    assert not any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)   # contact-only grasp

def test_camera_pose_from_config(model, data, cfg):
    mujoco.mj_forward(model, data)
    assert np.allclose(data.cam("overhead_camera").xpos, cfg.camera.position, atol=1e-9)

def test_start_keyframe_ball_on_board(model, data, cfg):  # ball center = board surface + radius at ball.start (board frame), 1e-4 m
```

`test_physics.py` (arm joints held at `initial_joints` by their position actuators):

```python
def test_grasp_holds_while_arm_moves():
    # 60 s sim: each joint follows 0.3*sin(2π·0.1·t) around initial_joints
    # board pose relative to tool0 changes by < 1e-3 m and < 0.5°
def test_release_drops_board():
    # set fingers_actuator to open; after 1 s, board z dropped > 0.1 m
def test_ball_rolls_and_walls_stop_it():
    # tilt the board 3° about y via wrist joints; ball moves toward the low side
    # and its x never passes the wall plane minus radius
def test_ball_falls_through_hole():
    # place ball 2 mm from the placeholder hole center, tilt toward it; ball z drops below board bottom within 2 s
```

- [ ] **Step 2: Run** `python3 -m pytest test/test_scene.py test/test_physics.py -v`. Expected: FAIL.
- [ ] **Step 3: Implement** `scene_builder.py`: include the Menagerie UR5e, attach the 2F-85 to the flange (Menagerie's attachment frame), insert the board fragment as a free body, the ball, the camera and `<option timestep cone noslip_iterations>` from config. Contact parameters live in `<default>` classes, not hard-coded per geom.
- [ ] **Step 4: Run tests.** Expected: PASS. If `test_grasp_holds_while_arm_moves` fails after tuning friction, finger force and `noslip_iterations`, stop and report to the owner (weld fallback needs approval).
- [ ] **Step 5: Commit** `[sim] MJCF scene: UR7e, 2F-85, board, ball, camera`

---

### Task 5: ros2_control bringup and interfaces

Depends on Task 1's decision.

**Files:**
- Create: `urdf/maze_sim.urdf.xacro` (in `maze_sim_description`), `src/maze_sim_bringup/{package.xml,setup.py,setup.cfg,resource/maze_sim_bringup}`, `launch/sim.launch.py`, `config/controllers.yaml`
- Test: `test/test_interfaces.py`

**Interfaces:**
- Consumes: `load_sim_config`, `write_scene` (Tasks 2, 4)
- Produces:
  - Launch arguments `config`, `viewer` (default `true`), `camera` (default `true`)
  - At launch: validate config (a `ConfigError` ends launch with its message), write the MJCF to `$ROS_HOME/maze_sim/scene.xml`, start `robot_state_publisher` with the xacro (`ur_type:=ur7e`), the plugin node, and spawners
  - `controllers.yaml`: controller names and parameters copied from `ur_robot_driver/config/ur_controllers.yaml`, `update_rate: 500`; `joint_state_broadcaster` with `joints:` the 6 UR joints; `gripper_controller` (`forward_command_controller/ForwardCommandController`, interface `position`) on the finger actuator

- [ ] **Step 1: Write failing `launch_testing` tests** (launch with `viewer:=false camera:=false`):

```python
EXPECTED_TOPICS = {
  "/joint_states": "sensor_msgs/msg/JointState",
  "/forward_velocity_controller/commands": "std_msgs/msg/Float64MultiArray",
  "/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
  "/scaled_joint_trajectory_controller/joint_trajectory": "trajectory_msgs/msg/JointTrajectory",
  "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage",
  "/clock": "rosgraph_msgs/msg/Clock",
}
def test_topics_and_types(node): ...
def test_trajectory_action_exists(node): ...   # FollowJointTrajectory server ready within 10 s
def test_controller_states(node):
    # list_controllers: scaled_joint_trajectory_controller active,
    # forward_velocity_controller and forward_position_controller inactive
def test_joint_states_are_ur_only(node):  # names == the 6 UR joints; rate 500 Hz ± 5% (sim time)
def test_tf_chain(node):  # base_link -> tool0 transform available
def test_bad_config_stops_launch(tmp_path):  # camera.fps: "sixty" -> process exits nonzero, stderr contains "camera.fps"
```

- [ ] **Step 2: Run** `colcon build --packages-up-to maze_sim_bringup && colcon test --packages-select maze_sim_bringup --pytest-args -k interfaces && colcon test-result --verbose`. Expected: FAIL.
- [ ] **Step 3: Implement** the xacro (include `ur_description` macro, replace the hardware with the plugin per Task 1), `controllers.yaml`, and `sim.launch.py`.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** `[sim] ros2_control bringup with UR7e controllers`

---

### Task 6: Arm control acceptance

Proves the controllers behave like the robot. Expect little new code beyond tests; fix configuration if a test fails.

**Files:**
- Create: `test/test_arm_control.py`, `test/lab7_style_node.py`

**Interfaces:**
- Consumes: the running sim from Task 5
- Produces: `lab7_style_node.py`, a node that switches to `forward_velocity_controller` with `ros2 control switch_controllers` (as lab 7's `switch_controllers`), reads `/joint_states`, and publishes `[0, 0, 0, 0, 0, 0.1]` on `/forward_velocity_controller/commands` for 2 s

- [ ] **Step 1: Write failing tests**

```python
def test_velocity_tracks():      # wrist_3 at 0.1 rad/s: measured velocity within 2% after 0.2 s
def test_position_reached():     # forward_position target +0.2 rad on elbow: |error| < 0.01 rad within 3 s
def test_trajectory_goal():      # FollowJointTrajectory, 3 waypoints over 6 s: result SUCCESSFUL
def test_switch_both_ways():     # trajectory -> velocity -> trajectory with switch_controllers
def test_lab7_style_node_unchanged():   # run lab7_style_node.py; wrist_3 moved ≈ 0.2 rad (±10%)
def test_wrong_length_command_ignored():  # 5-value array: no joint moves > 1e-3 rad in 1 s; sim still publishing /clock
```

- [ ] **Step 2: Run** `colcon test --packages-select maze_sim_bringup --pytest-args -k arm_control`. Expected: FAIL (file missing), then failures that point at configuration.
- [ ] **Step 3: Fix** controller or actuator gains in the MJCF defaults or `controllers.yaml` until behavior matches; do not add custom controllers.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** `[sim] arm control acceptance tests`

---

### Task 7: Camera and sim helpers

Depends on Task 1's decision for how camera images and body poses leave the plugin.

**Files:**
- Create: `maze_sim_bringup/sim_tools_node.py` (only for what the plugin does not provide); camera block in `sim.launch.py`
- Test: `test/test_camera.py`, `test/test_sim_tools.py`

**Interfaces:**
- Consumes: `pinhole_intrinsics`, `look_at_quaternion` (Task 2); camera `overhead_camera`, bodies `board`, `ball`, keyframe `start` (Task 4)
- Produces:
  - `/camera/image_raw` (`rgb8`), `/camera/camera_info`, frame `camera_optical_frame`, static TF `world` → `camera_optical_frame` from config
  - `/sim/ball/pose` and `/sim/board/pose` (`geometry_msgs/PoseStamped`, frame `world`)
  - `/sim/reset` (`std_srvs/Trigger`): restore keyframe `start`, zero velocities, finger target `gripper.closed_position`, controller references reset to `initial_joints`; then publishes `std_msgs/Empty` on `/sim/reset_done`

- [ ] **Step 1: Write failing tests**

```python
def test_camera_rate():          # /camera/image_raw at 60 fps ± 5% (sim time), 1280x720 rgb8
def test_camera_info_matches():  # K == pinhole_intrinsics(1280, 720, 70.42); D all zero; frame_id camera_optical_frame
def test_camera_tf_matches_config():  # world->camera_optical_frame == look_at_quaternion(cfg.camera.position, cfg.camera.look_at, cfg.camera.up) within 1e-6
def test_board_in_frame():       # project the 4 board corners (from /sim/board/pose and extent) with K: all inside the image
def test_ball_pose_matches_start():   # after reset, /sim/ball/pose == board pose ∘ ball.start (1e-4 m)
def test_reset_is_repeatable():  # reset, send the same 6 s FollowJointTrajectory tilt, record ball xy at 10 Hz; twice; max difference < 1e-3 m
```

- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement** using the plugin's `CameraPlugin` config if Task 1 passed item 2 (topic names remapped to `/camera/...`, frame name overridden), else per the owner's Task 1 decision. When images are published below `fps`, log the achieved rate (spec section 9: the camera rate drops before the physics rate).
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** `[sim] overhead camera and /sim helpers`

---

### Task 8: Gripper service

**Files:**
- Create: `maze_sim_bringup/gripper_node.py`; add it to `sim.launch.py`
- Test: `test/test_gripper.py`

**Interfaces:**
- Consumes: `/gripper_controller/commands` (`Float64MultiArray`, one value) from Task 5; `gripper.closed_position` from config; `/sim/board/pose` and `/sim/reset` from Task 7
- Produces: service `/toggle_gripper` (`std_srvs/Trigger`); starts in state `closed`; each call flips state and publishes the target (`closed_position` or `0.0`); `response.success = True`, `response.message` is `"opened"` or `"closed"`. Subscribes to `/sim/reset_done` and sets its state to `closed` on each message

- [ ] **Step 1: Write failing tests**

```python
def test_toggle_reports_state():        # first call "opened", second "closed"
def test_open_drops_board():            # after "opened", /sim/board/pose z drops > 0.1 m within 1 s
def test_joint_states_still_ur_only():  # gripper joint absent from /joint_states
def test_reset_after_release_regrasps():  # open, wait 1 s, /sim/reset: board back in grasp, holds 5 s (< 1 mm vs tool0); next toggle says "opened"
```

- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement** `gripper_node.py` (rclpy, `use_sim_time`).
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Commit** `[sim] /toggle_gripper service`

---

### Task 9: Real-time check and README

**Files:**
- Modify: `maze_sim_bringup/sim_tools_node.py` (real-time factor log), `README.md`
- Test: `test/test_sim_tools.py` (add)

**Interfaces:**
- Produces: every 10 s the sim logs `real-time factor: <x.xx>` (sim time elapsed ÷ wall time elapsed)

- [ ] **Step 1: Write failing tests**

```python
def test_rtf_logged():            # launch output contains "real-time factor:" within 15 s
def test_sim_time_has_no_gaps():  # /clock deltas are multiples of 0.002 s; no delta > 0.1 s of sim time
```

- [ ] **Step 2: Run.** Expected: FAIL.
- [ ] **Step 3: Implement** the log. Measure by hand with `viewer:=true camera:=true` on the owner's machine; record the factor in the README. Spec target: ≥ 1.0.
- [ ] **Step 4: Run tests.** Expected: PASS.
- [ ] **Step 5: Add README section** "Simulation": prerequisites, build, `ros2 launch maze_sim_bringup sim.launch.py`, the interface table from spec section 5, board export rules from spec section 6, `sim.yaml` reference.
- [ ] **Step 6: Commit** `[sim] real-time factor log and README`

---

### Task 10: Clean-container check

Proves a teammate can go from a fresh clone to a running sim with only podman and distrobox on their machine.

**Files:**
- Modify: `docker/Containerfile` (any dependency Tasks 1–9 added), `README.md`

- [ ] **Step 1: Fold dependencies in**: every package installed by hand during Tasks 1–9 goes into the Containerfile; add `rosdep install --from-paths src -y --ignore-src` against copies of the two `package.xml` files.
- [ ] **Step 2: Rebuild from scratch and run everything** (owner, normal terminal):

```bash
distrobox rm -f maze-sim && podman rmi -f maze-sim && ./docker/build.sh
distrobox assemble create --file docker/distrobox.ini
distrobox enter maze-sim -- bash -lc "cd ~/Documents/UCB/eecs_106/ee206-project && rm -rf build install log && colcon build && source install/setup.bash && colcon test && colcon test-result --verbose"
```

Expected: all tests pass; `ros2 launch maze_sim_bringup sim.launch.py viewer:=true` opens the MuJoCo viewer from inside the container.
- [ ] **Step 3: Host unchanged**: `git status` shows only intended repo changes, and `ls -la ~ ~/.local/lib/python3.10/site-packages` matches a listing taken before Task 0 (no new dotfiles or user packages).
- [ ] **Step 4: Commit** `[sim] container includes all S1 dependencies`

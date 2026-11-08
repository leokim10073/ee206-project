# mujoco_ros2_control capability check on Humble

Date: 2026-11-08. Run inside the `maze-sim` container (Task 0) with apt packages
`ros-humble-mujoco-ros2-control` 0.1.2, `ros-humble-mujoco-ros2-control-plugins` 0.1.2,
`ros-humble-mujoco-vendor` 0.1.1. Docs read from the upstream repo at tag `0.1.2`
(`mujoco_ros2_control/docs/hardware_interface.rst`, `mujoco_ros2_control_plugins/doc/plugins.rst`).
Live checks used `mujoco_ros2_control_demos` 0.1.2 built in a scratch workspace and
`01_basic_robot.launch.py headless:=true`.

## Results

| # | Capability | Result | Evidence |
|---|---|---|---|
| 1 | Runs on Humble; MuJoCo version | Pass | Demo launched headless; controllers active. The plugin links `libmujoco.so.3.12.0` from `mujoco_vendor`; the image's Python `mujoco` was 3.4.0, so it is re-pinned to 3.12.0. |
| 2 | Camera images and `CameraInfo` | Pass | `CameraPlugin` publishes `/camera/color/image_raw` and `/camera/color/camera_info` (frame `camera_color_optical_frame`). Topic and frame names are set per camera in the plugin YAML; rate is `camera_publish_rate` (one rate for all cameras). The demo's 6 Hz setting measured 8–9 Hz in wall time. Rendering used GLFW; EGL is the headless fallback. 60 Hz at 1280×720 is checked in Task 7 on our scene. |
| 3 | Gripper through a ros2_control command interface | Pass | Demo `gripper_controller` (`forward_command_controller`) is active. The docs require a position actuator on a tendon named after the controllable joint, with an equality constraint for the mimic finger: the same structure as the Menagerie 2F-85. |
| 4 | Free-body pose out, model reset in | Pass | `FreeJointStatePublisherPlugin` publishes all free bodies on `free_joint_states` (`mujoco_ros2_control_msgs/FreeJointStateArray`, configurable rate, frame and body list). `/mujoco_ros2_control_node/reset_world` returned `success=True`; it accepts a keyframe name and per-body overrides. `set_free_joint_state`, `set_pause`, `step_simulation` also exist. The hardware param `initial_keyframe` sets the start state. |
| 5 | UR7e vs UR5e parameters | Identical | `diff` of `default_kinematics.yaml`, `physical_parameters.yaml` and `joint_limits.yaml` between `ur_description/config/ur5e` and `ur7e`: no differences. The Menagerie UR5e is the right arm model. |
| 6 | `/clock` | Pass | Published by the plugin from the physics clock. |

## Findings that change the implementation

- **Arm actuators.** A MuJoCo `position` actuator supports only the ros2_control `position` command interface; `velocity` on it is not supported. Our arm needs position (trajectory and forward position controllers) and velocity (forward velocity controller) on the same joints. Supported pattern: `motor` actuators with plugin PIDs (`pids_config_file`), which accept both position and velocity command interfaces and allow switching at runtime. The Menagerie UR5e's `general` position servos are replaced by `motor` actuators with the UR joint torque limits.
- **Service and node names.** The node is `/mujoco_ros2_control_node`; services are `/mujoco_ros2_control_node/reset_world` etc. Upstream docs say to deactivate joint controllers before `reset_world`, or they snap the arm back.
- **Real-time scheduling.** The controller manager warns it cannot enable FIFO scheduling inside the container. Harmless for the sim; noted for the real-time check.
- **Camera TF.** The plugin sets `frame_id` on the messages but the camera TF must still be published (our static transform from `sim.yaml`).

## Decision needed

None: every item passed. The two choices above (motor actuators + PIDs; Python `mujoco` 3.12.0) follow from the docs and are recorded as rulings in the execution ledger.

"""Compose the MuJoCo scene: Menagerie UR5e (= UR7e kinematics) with a Robotiq 2F-85 on the
flange, the maze board held in the gripper, the ball, the overhead camera, and keyframe "start".

The arm's position servos are replaced by torque (motor) actuators named after the joints, so
mujoco_ros2_control's PIDs can serve both position and velocity command interfaces. The gripper
actuator is a position servo on the finger tendon, named after the driver joint as the plugin
requires for tendon actuators. The grasp is contact only.
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

from .board_assets import board_mjcf_fragment, load_board_assets
from .camera_math import matrix_to_quat, mujoco_camera_quaternion
from .config import load_sim_config

UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
UR_TORQUE_LIMITS = (150.0, 150.0, 150.0, 28.0, 28.0, 28.0)  # N m, UR7e/UR5e datasheet
GRIPPER_PREFIX = "gripper_"
GRIPPER_ACTUATOR = GRIPPER_PREFIX + "right_driver_joint"
BOARD_PREFIX = "maze_"
GRIPPER_KP = 100.0
GRIPPER_KV = 10.0
# Tendon force (N m on the driver joints) per newton of finger normal force on the handle;
# checked by test_physics.test_finger_force_matches_limit.
GRIPPER_TENDON_PER_NEWTON = 0.114
SETTLE_S = 0.5  # pre-simulation that closes the fingers on the handle (board pinned)
LOAD_SETTLE_S = 1.0  # then the board hangs in the grasp under its own weight


def _menagerie_dir():
    here = Path(__file__).resolve().parents[1] / "mjcf" / "menagerie"
    if here.is_dir():
        return here
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("maze_sim_description")) / "mjcf" / "menagerie"


def _absolute_meshes(spec, mesh_dir):
    for mesh in spec.meshes:
        mesh.file = str(Path(mesh_dir).resolve() / mesh.file)
    spec.meshdir = ""


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx  # ROS fixed-axis roll, pitch, yaw


def gripper_command(model, closed):
    """ctrl value that fully closes (force-limited) or fully opens the fingers."""
    lo, hi = model.actuator(GRIPPER_ACTUATOR).ctrlrange
    return float(hi if closed else lo)


def _compose(cfg, board):
    menagerie = _menagerie_dir()
    spec = mujoco.MjSpec.from_file(str(menagerie / "universal_robots_ur5e" / "ur5e.xml"))
    _absolute_meshes(spec, menagerie / "universal_robots_ur5e" / "assets")
    spec.modelname = "maze_sim"
    spec.option.timestep = cfg.physics.timestep
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.noslip_iterations = cfg.physics.noslip_iterations
    spec.option.impratio = 10.0  # the 2F-85 model's value; set before attaching so the options agree
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for key in list(spec.keys):
        spec.delete(key)
    for act in list(spec.actuators):
        spec.delete(act)
    for joint, limit in zip(UR_JOINTS, UR_TORQUE_LIMITS):
        act = spec.add_actuator(name=joint, target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_motor()
        act.ctrllimited = act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.ctrlrange = act.forcerange = [-limit, limit]

    flange = spec.site("attachment_site")
    spec.body("wrist_3_link").add_site(name="tool0", pos=flange.pos, quat=flange.quat)

    gripper = mujoco.MjSpec.from_file(str(menagerie / "robotiq_2f85" / "2f85.xml"))
    _absolute_meshes(gripper, menagerie / "robotiq_2f85" / "assets")
    for act in list(gripper.actuators):
        gripper.delete(act)
    act = gripper.add_actuator(name="right_driver_joint", target="split", trntype=mujoco.mjtTrn.mjTRN_TENDON)
    act.set_to_position(kp=GRIPPER_KP, kv=GRIPPER_KV)
    act.ctrllimited = act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    act.ctrlrange = [0.0, 0.8]
    f = cfg.gripper.force_limit * GRIPPER_TENDON_PER_NEWTON
    act.forcerange = [-f, f]
    for geom in gripper.geoms:
        if "pad" in geom.name:
            geom.friction = cfg.board.surface_friction  # pads keep priority 1 and their stiff solref
    flange.attach_body(gripper.body("base_mount"), GRIPPER_PREFIX, "")

    board_spec = mujoco.MjSpec.from_string(
        "<mujoco>" + board_mjcf_fragment(board, cfg.board.mass, cfg.board.surface_friction) + "</mujoco>"
    )
    _absolute_meshes(board_spec, board.visual_meshes[0].parent)
    # An empty prefix would copy the board's main default as an unnamed class, which to_xml
    # cannot write back; attach with a prefix and strip it from the named elements.
    spec.worldbody.add_frame().attach_body(board_spec.body("board"), BOARD_PREFIX, "")
    for element in [*spec.meshes, *spec.bodies, *spec.joints, *spec.geoms]:
        if element.name.startswith(BOARD_PREFIX):
            element.name = element.name[len(BOARD_PREFIX):]
    for geom in spec.geoms:
        if geom.meshname.startswith(BOARD_PREFIX):
            geom.meshname = geom.meshname[len(BOARD_PREFIX):]

    ball_cls = spec.add_default("ball", spec.default)
    ball_cls.geom.friction = cfg.ball.friction
    ball_cls.geom.condim = 6
    ball_cls.geom.priority = 1  # ball friction wins against the board's
    ball_cls.geom.solref = [2 * cfg.physics.timestep, 1.0]  # stiffest stable contact: walls stop the ball
    ball = spec.worldbody.add_body(name="ball")
    ball.add_freejoint(name="ball_free")
    ball.add_geom(
        default=ball_cls, name="ball", type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[cfg.ball.radius, 0, 0], mass=cfg.ball.mass, rgba=[0.75, 0.75, 0.8, 1],
    )

    spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05], rgba=[0.5, 0.5, 0.5, 1])
    spec.worldbody.add_light(pos=[0.5, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)

    w, h = cfg.camera.resolution
    hfov = np.radians(cfg.camera.hfov_deg)
    spec.worldbody.add_camera(
        name="overhead_camera",
        pos=cfg.camera.position,
        quat=mujoco_camera_quaternion(cfg.camera.position, cfg.camera.look_at, cfg.camera.up),
        fovy=float(np.degrees(2 * np.arctan(np.tan(hfov / 2) * h / w))),
    )

    return spec


def _handle_target(data, cfg):
    """Handle frame in world: tool0 * grasp_offset. Its axes are the board's axes."""
    R_t = data.site("tool0").xmat.reshape(3, 3)
    return data.site("tool0").xpos + R_t @ np.asarray(cfg.board.grasp_offset.xyz), R_t @ _rpy_matrix(cfg.board.grasp_offset.rpy)


def _grasp_and_settle(model, cfg, handle, p_h, R_h):
    """Pin the board with its handle at (p_h, R_h), close the fingers on it, then let the board
    hang in the grasp under its weight. Arm held at initial_joints, ball parked away."""
    data = mujoco.MjData(model)
    arm_q = [model.joint(j).qposadr[0] for j in UR_JOINTS]
    arm_v = [model.joint(j).dofadr[0] for j in UR_JOINTS]
    b_adr, b_dof = model.joint("board_free").qposadr[0], model.joint("board_free").dofadr[0]
    ball_adr, ball_dof = model.joint("ball_free").qposadr[0], model.joint("ball_free").dofadr[0]
    lo, hi = model.actuator(GRIPPER_ACTUATOR).ctrlrange
    for side in ("right", "left"):
        data.joint(f"{GRIPPER_PREFIX}{side}_driver_joint").qpos = lo + cfg.gripper.closed_position * (hi - lo)
    board_q = np.concatenate([p_h - R_h @ handle, matrix_to_quat(R_h)])
    data.ctrl[model.actuator(GRIPPER_ACTUATOR).id] = gripper_command(model, closed=True)

    def hold(pin_board):
        data.qpos[arm_q] = cfg.robot.initial_joints
        data.qvel[arm_v] = 0
        data.qpos[ball_adr : ball_adr + 7] = (0, 0, -1.0, 1, 0, 0, 0)
        data.qvel[ball_dof : ball_dof + 6] = 0
        if pin_board:
            data.qpos[b_adr : b_adr + 7] = board_q
            data.qvel[b_dof : b_dof + 6] = 0

    for pin_board, seconds in ((True, SETTLE_S), (False, LOAD_SETTLE_S)):
        for _ in range(int(round(seconds / model.opt.timestep))):
            hold(pin_board)
            mujoco.mj_step(model, data)
    hold(False)
    data.qvel[:] = 0
    mujoco.mj_kinematics(model, data)
    return data


def _start_keyframe(model, cfg, board):
    """Arm at initial_joints, fingers closed on the handle with the board pinned at grasp_offset,
    then the board settled in the grasp under its weight (the fingers give a few degrees; the
    default initial_joints are chosen so the settled board is level), ball resting at ball.start
    on the board surface."""
    handle = np.asarray(board.handle_center_m)
    data = mujoco.MjData(model)
    data.qpos[[model.joint(j).qposadr[0] for j in UR_JOINTS]] = cfg.robot.initial_joints
    mujoco.mj_kinematics(model, data)
    data = _grasp_and_settle(model, cfg, handle, *_handle_target(data, cfg))
    p_b, R_b = data.body("board").xpos.copy(), data.body("board").xmat.reshape(3, 3).copy()
    ball = model.joint("ball_free").qposadr[0]
    data.qpos[ball : ball + 7] = np.concatenate([p_b + R_b @ np.array([*cfg.ball.start, cfg.ball.radius]), [1, 0, 0, 0]])
    return data.qpos.copy(), data.ctrl.copy()


def _scene_spec(cfg, board_dir):
    board = load_board_assets(board_dir)
    spec = _compose(cfg, board)
    model = spec.compile()
    qpos, ctrl = _start_keyframe(model, cfg, board)
    spec.add_key(name="start", qpos=qpos, ctrl=ctrl)
    return spec


def build_scene(cfg, board_dir):
    """MJCF text of the whole scene, with absolute mesh paths."""
    return _scene_spec(cfg, board_dir).to_xml()


def write_scene(cfg, board_dir, out_path):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(build_scene(cfg, board_dir))
    return out_path


def main():
    p = argparse.ArgumentParser(description="Write the MuJoCo scene for a sim.yaml.")
    p.add_argument("config")
    p.add_argument("board_dir")
    p.add_argument("out")
    a = p.parse_args()
    print(write_scene(load_sim_config(a.config), a.board_dir, a.out))


if __name__ == "__main__":
    main()

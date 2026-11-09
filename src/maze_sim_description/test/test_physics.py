"""Pure-MuJoCo physics checks. The arm is held by a PD loop plus the remaining gravity and
Coriolis torques (MuJoCo already cancels the arm and gripper weight) on its motor actuators:
the job ros2_control's PIDs do in the running sim. The grasp weld follows grasp_weld_engaged
every step, as GraspWeldPlugin makes it in the running sim."""

import mujoco
import numpy as np
import trimesh

from maze_sim_description.scene_builder import (
    GRASP_WELD, GRIPPER_ACTUATOR, UR_JOINTS, grasp_weld_engaged, gripper_command,
)

KP, KD = 2000.0, 100.0


def _arm_ids(model):
    return (
        np.array([model.joint(j).qposadr[0] for j in UR_JOINTS]),
        np.array([model.joint(j).dofadr[0] for j in UR_JOINTS]),
        np.array([model.actuator(j).id for j in UR_JOINTS]),
    )


def _run(model, data, seconds, q_target, grip_closed=True, on_step=None):
    qa, da, aa = _arm_ids(model)
    weld = model.equality(GRASP_WELD).id
    data.ctrl[model.actuator(GRIPPER_ACTUATOR).id] = gripper_command(model, grip_closed)
    for _ in range(int(round(seconds / model.opt.timestep))):
        data.eq_active[weld] = grasp_weld_engaged(model, data, bool(data.eq_active[weld]))
        qd = q_target(data.time)
        data.ctrl[aa] = KP * (qd - data.qpos[qa]) - KD * data.qvel[da] + data.qfrc_bias[da] - data.qfrc_gravcomp[da]
        mujoco.mj_step(model, data)
        if on_step:
            on_step(data)


def _rel_pose(data, handle):
    """Handle center position and board rotation in the tool0 frame."""
    R_t = data.site("tool0").xmat.reshape(3, 3)
    R_b = data.body("board").xmat.reshape(3, 3)
    return R_t.T @ (data.body("board").xpos + R_b @ handle - data.site("tool0").xpos), R_t.T @ R_b


def _to_board(data, p):
    return data.body("board").xmat.reshape(3, 3).T @ (p - data.body("board").xpos)


def _place_ball(model, data, xy_board):
    R = data.body("board").xmat.reshape(3, 3)
    r = model.geom_size[model.body("ball").geomadr[0]][0]
    adr = model.joint("ball_free").qposadr[0]
    data.qpos[adr : adr + 3] = data.body("board").xpos + R @ np.array([*xy_board, r])
    data.qvel[model.joint("ball_free").dofadr[0] : model.joint("ball_free").dofadr[0] + 6] = 0
    mujoco.mj_forward(model, data)


def _tilt_gravity(model, axis, deg):
    """Tilting gravity is equivalent to tilting the held board, without moving the arm."""
    g = np.array([0, 0, -9.81])
    a = np.radians(deg)
    if axis == "x":
        R = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    else:
        R = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    model.opt.gravity[:] = R @ g


def _handle(model):
    from conftest import PKG
    from maze_sim_description.board_assets import load_board_assets

    return np.array(load_board_assets(PKG / "meshes" / "board").handle_center_m)


def test_grasp_holds_while_arm_moves(model, data, cfg):
    # 0.1 rad on every joint tilts the board well past what maze play needs, and keeps the
    # 0.5 m board clear of the table and the arm.
    q0 = np.array(cfg.robot.initial_joints)
    h = _handle(model)
    p0, R0 = _rel_pose(data, h)
    worst = [0.0, 0.0]

    def track(d):
        p, R = _rel_pose(d, h)
        worst[0] = max(worst[0], np.linalg.norm(p - p0))
        worst[1] = max(worst[1], np.degrees(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1))))

    _run(model, data, 60.0, lambda t: q0 + 0.1 * np.sin(2 * np.pi * 0.1 * t), on_step=track)
    assert data.eq_active[model.equality(GRASP_WELD).id]
    # Contacts alone let the board creep without bound (0.84° here, growing ~0.014°/s). Welded,
    # it only flexes against the finger friction: 0.24 mm / 0.07° here, levelling off near
    # 0.6 mm / 0.17° over 5 minutes.
    assert worst[0] < 0.5e-3, f"board slid {worst[0] * 1000:.2f} mm in the grasp"
    assert worst[1] < 0.15, f"board rotated {worst[1]:.3f} deg in the grasp"


def test_finger_force_matches_limit(model, data, cfg):
    q0 = np.array(cfg.robot.initial_joints)
    _run(model, data, 0.5, lambda t: q0)
    handle = {i for i in range(model.ngeom) if model.geom(i).name.startswith("board_col_handle")}
    total = {"left": 0.0, "right": 0.0}
    f = np.zeros(6)
    for i, c in enumerate(data.contact[: data.ncon]):
        names = {model.geom(c.geom1).name, model.geom(c.geom2).name}
        if {c.geom1, c.geom2} & handle:
            for side in total:
                if any(n.startswith(f"gripper_{side}_pad") for n in names):
                    mujoco.mj_contactForce(model, data, i, f)
                    total[side] += f[0]
    for side, force in total.items():
        assert abs(force - cfg.gripper.force_limit) < 0.2 * cfg.gripper.force_limit, f"{side} finger {force:.0f} N"


def test_release_drops_board(model, data, cfg):
    q0 = np.array(cfg.robot.initial_joints)
    z0 = data.body("board").xpos[2]
    _run(model, data, 1.0, lambda t: q0, grip_closed=False)
    assert z0 - data.body("board").xpos[2] > 0.1


def _first_wall_toward_plus_y(xy):
    """Inner face (board y) of the nearest wall at +y from xy, from the board's wall meshes."""
    from conftest import PKG

    faces = []
    for p in (PKG / "meshes" / "board").glob("visual_wall_*.obj"):
        (x0, y0, z0), (x1, _, z1) = trimesh.load(p, force="mesh").bounds
        if x0 <= xy[0] <= x1 and y0 > xy[1] and z1 > 0.005:  # skips the goal pocket's floor disk
            faces.append(y0)
    return min(faces)


def test_ball_rolls_and_walls_stop_it(model, data, cfg):
    q0 = np.array(cfg.robot.initial_joints)
    start = tuple(cfg.ball.start)
    wall_y = _first_wall_toward_plus_y(start)
    _place_ball(model, data, start)
    r = cfg.ball.radius
    max_y = [-1.0]
    gravity = model.opt.gravity.copy()
    try:
        _tilt_gravity(model, "x", 3.0)  # board +y downhill (board axes are world axes at start)
        _run(model, data, 3.0, lambda t: q0,
             on_step=lambda d: max_y.__setitem__(0, max(max_y[0], _to_board(d, d.body("ball").xpos)[1])))
    finally:
        model.opt.gravity[:] = gravity
    assert max_y[0] > wall_y - r - 0.003, "ball did not roll to the wall"
    assert max_y[0] < wall_y - r + 1e-3, "ball passed through the wall"


def test_goal_pocket_holds_ball(model, data, cfg):
    # the pocket must stay open after convex decomposition: the ball sinks below the floor
    # and a tilt that rolls it across the open board keeps it inside the pocket
    q0 = np.array(cfg.robot.initial_joints)
    goal = np.array(cfg.board.goal_hole.center)
    _place_ball(model, data, tuple(goal))
    track = {"z": 1.0, "off": 0.0}

    def on_step(d):
        b = _to_board(d, d.body("ball").xpos)
        track["z"] = min(track["z"], b[2])
        if d.time > 0.5:
            track["off"] = max(track["off"], np.linalg.norm(b[:2] - goal))

    gravity = model.opt.gravity.copy()
    try:
        _tilt_gravity(model, "y", 3.0)
        _run(model, data, 2.0, lambda t: q0, on_step=on_step)
    finally:
        model.opt.gravity[:] = gravity
    assert track["z"] < cfg.ball.radius - 0.002, "ball did not sink into the pocket"
    assert track["off"] < cfg.board.goal_hole.radius, "ball rolled out of the pocket"

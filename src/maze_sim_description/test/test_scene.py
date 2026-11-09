import mujoco
import numpy as np
import pytest

from maze_sim_description.scene_builder import (
    GRASP_WELD, GRASP_WELD_ANG_TOL_DEG, GRASP_WELD_POS_TOL, GRIPPER_ACTUATOR, UR_JOINTS, grasp_weld_engaged,
    gripper_command,
)


def _board_frame(data, p_world):
    R = data.body("board").xmat.reshape(3, 3)
    return R.T @ (np.asarray(p_world) - data.body("board").xpos)


def test_compiles_with_names(model, cfg):
    for j in UR_JOINTS:
        model.joint(j)
        assert model.actuator(j).trntype == mujoco.mjtTrn.mjTRN_JOINT
        assert model.actuator(j).gaintype == mujoco.mjtGain.mjGAIN_FIXED  # motor: torque in, PIDs in ros2_control
        assert model.actuator(j).biastype == mujoco.mjtBias.mjBIAS_NONE
    model.joint(GRIPPER_ACTUATOR)  # tendon actuator named after its joint (plugin requirement)
    model.actuator(GRIPPER_ACTUATOR)
    model.camera("overhead_camera"); model.body("board"); model.body("ball"); model.site("tool0")
    assert model.joint("ball_free").type == mujoco.mjtJoint.mjJNT_FREE
    assert model.opt.timestep == 0.002
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.noslip_iterations == cfg.physics.noslip_iterations


def test_camera_pose_from_config(model, data, cfg):
    assert np.allclose(data.cam("overhead_camera").xpos, cfg.camera.position, atol=1e-9)
    forward = -data.cam("overhead_camera").xmat.reshape(3, 3)[:, 2]  # MuJoCo cameras look along -z
    want = np.subtract(cfg.camera.look_at, cfg.camera.position)
    assert np.allclose(forward, want / np.linalg.norm(want), atol=1e-6)
    w, h = cfg.camera.resolution
    hfov = 2 * np.degrees(np.arctan(np.tan(np.radians(model.cam_fovy[0] / 2)) * w / h))
    assert hfov == pytest.approx(cfg.camera.hfov_deg, abs=1e-4)  # cam_fovy is float32 in XML round-trip


def test_start_keyframe_arm_at_initial_joints(model, data, cfg):
    q = [data.joint(j).qpos[0] for j in UR_JOINTS]
    assert np.allclose(q, cfg.robot.initial_joints, atol=1e-9)


def test_start_keyframe_ball_on_board(model, data, cfg):
    p = _board_frame(data, data.body("ball").xpos)
    assert np.allclose(p, [*cfg.ball.start, cfg.ball.radius], atol=1e-4)


def test_start_keyframe_board_in_grasp(model, data, cfg):
    # the handle center sits at grasp_offset in the tool0 frame, within what the fingers give under the board's weight
    from maze_sim_description.board_assets import load_board_assets
    from conftest import PKG

    handle = load_board_assets(PKG / "meshes" / "board").handle_center_m
    R_b = data.body("board").xmat.reshape(3, 3)
    h_world = data.body("board").xpos + R_b @ handle
    R_t = data.site("tool0").xmat.reshape(3, 3)
    h_tool = R_t.T @ (h_world - data.site("tool0").xpos)
    assert np.allclose(h_tool, cfg.board.grasp_offset.xyz, atol=5e-3)
    assert R_b[2, 2] > np.cos(np.radians(0.2))  # board level within 0.2°


def test_no_arm_contact_with_board_at_start(model, data):
    board = model.body("board").id
    arm = {model.body(n).id for n in ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link")}
    for c in data.contact[: data.ncon]:
        bodies = {model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]}
        assert not (board in bodies and bodies & arm)


def test_write_scene(cfg, tmp_path):
    from maze_sim_description.scene_builder import write_scene
    from conftest import PKG

    out = write_scene(cfg, PKG / "meshes" / "board", tmp_path / "scene.xml")
    m = mujoco.MjModel.from_xml_path(str(out))
    assert m.nkey >= 1 and m.key("start").id >= 0


def test_write_scene_from_relative_board_dir(cfg, tmp_path, monkeypatch):
    from maze_sim_description.scene_builder import write_scene
    from conftest import PKG

    monkeypatch.chdir(PKG)
    out = write_scene(cfg, "meshes/board", tmp_path / "scene.xml")
    monkeypatch.chdir(tmp_path)
    mujoco.MjModel.from_xml_path(str(out))


def test_arm_and_gripper_gravity_compensated(model):
    # the UR controller cancels the weight of the arm and its tool; ros2_control's PIDs have no gravity term
    for j in UR_JOINTS:
        assert model.jnt_actgravcomp[model.joint(j).id]  # counts against the joint torque limit
    for b in ("shoulder_link", "wrist_3_link", "gripper_base", "gripper_left_pad"):
        assert model.body_gravcomp[model.body(b).id] == 1.0
    for b in ("board", "ball"):
        assert model.body_gravcomp[model.body(b).id] == 0.0


def test_world_is_ros_base_link(model):
    # ur_description at zero joints: tool0 in base_link is (-(a2+a3), d4+d6, d1-d5) for the UR5e/UR7e
    d = mujoco.MjData(model)
    mujoco.mj_kinematics(model, d)
    np.testing.assert_allclose(d.site("tool0").xpos, [0.8172, 0.2329, 0.0628], atol=2e-3)


def test_camera_resolution_and_offscreen_buffer(model, cfg):
    w, h = cfg.camera.resolution
    assert tuple(model.cam_resolution[model.camera("overhead_camera").id]) == (w, h)
    assert model.vis.global_.offwidth >= w and model.vis.global_.offheight >= h


def test_board_units_must_match_meshes(cfg):
    import dataclasses

    from maze_sim_description.scene_builder import build_scene

    from conftest import PKG

    wrong = dataclasses.replace(cfg, board=dataclasses.replace(cfg.board, units="m"))  # meshes were built from mm
    with pytest.raises(ValueError, match="board.units"):
        build_scene(wrong, PKG / "meshes" / "board")


def test_aruco_markers_on_board(model):
    # the four corner stickers perception will look for: drawn on the board, never touched
    names = sorted(model.geom(i).name for i in range(model.ngeom) if model.geom(i).name.startswith("board_aruco_"))
    assert names == ["board_aruco_BL", "board_aruco_BR", "board_aruco_TL", "board_aruco_TR"]
    for n in names:
        g = model.geom(n)
        assert model.body(g.bodyid[0]).name == "board"
        assert (g.contype[0], g.conaffinity[0]) == (0, 0)
        assert model.mat_texid[g.matid[0]][int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] >= 0


def test_grasp_weld_off_until_the_plugin_engages_it(model):
    # MuJoCo's soft friction lets a held board creep; while the gripper is closed on it,
    # GraspWeldPlugin switches this weld on (the rule is grasp_weld_engaged)
    weld = model.equality(GRASP_WELD)
    assert weld.type[0] == mujoco.mjtEq.mjEQ_WELD
    assert model.body(weld.obj1id[0]).name == "wrist_3_link" and model.body(weld.obj2id[0]).name == "board"
    assert not model.eq_active0[weld.id]
    assert int(sum(model.eq_type == mujoco.mjtEq.mjEQ_WELD)) == 1
    # what GraspWeldPlugin reads from the model: the tolerances and the gripper actuator
    tol = model.numeric("grasp_weld")
    assert list(model.numeric_data[tol.adr[0] : tol.adr[0] + tol.size[0]]) == [GRASP_WELD_POS_TOL, GRASP_WELD_ANG_TOL_DEG]
    t = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "grasp_weld_actuator")
    adr, size = model.text_adr[t], model.text_size[t]
    assert bytes(model.text_data[adr : adr + size - 1]).decode() == GRIPPER_ACTUATOR


def test_grasp_weld_holds_board_at_grasp_offset(model, data, cfg):
    # the weld's target is the board with its handle exactly at grasp_offset in tool0
    from conftest import PKG
    from maze_sim_description.board_assets import load_board_assets

    handle = np.array(load_board_assets(PKG / "meshes" / "board").handle_center_m)
    rel = model.eq_data[model.equality(GRASP_WELD).id]
    R_w = data.body("wrist_3_link").xmat.reshape(3, 3)
    p_w = data.body("wrist_3_link").xpos
    R_rel = np.zeros(9)
    mujoco.mju_quat2Mat(R_rel, rel[6:10])
    p_board, R_board = p_w + R_w @ rel[3:6], R_w @ R_rel.reshape(3, 3)
    R_t = data.site("tool0").xmat.reshape(3, 3)
    h_tool = R_t.T @ (p_board + R_board @ handle - data.site("tool0").xpos)
    assert np.allclose(h_tool, cfg.board.grasp_offset.xyz, atol=1e-6)


def test_grasp_weld_rule(model, data):
    open_cmd, closed_cmd = gripper_command(model, False), gripper_command(model, True)
    act = model.actuator(GRIPPER_ACTUATOR).id
    d = mujoco.MjData(model)
    d.qpos[:], d.ctrl[:] = data.qpos, data.ctrl
    mujoco.mj_forward(model, d)
    d.ctrl[act] = closed_cmd
    assert grasp_weld_engaged(model, d, engaged=False)  # closed, board in the grasp
    d.ctrl[act] = open_cmd
    assert not grasp_weld_engaged(model, d, engaged=True)  # opening always releases
    d.ctrl[act] = closed_cmd
    adr = model.joint("board_free").qposadr[0]
    d.qpos[adr + 2] -= 0.02  # board 2 cm below the grasp, e.g. dropped and closing again
    mujoco.mj_forward(model, d)
    assert not grasp_weld_engaged(model, d, engaged=False)
    assert grasp_weld_engaged(model, d, engaged=True)  # once on, it stays on while closed

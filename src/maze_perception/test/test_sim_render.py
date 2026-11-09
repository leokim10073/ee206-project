"""Board pose from the sim's own overhead camera image, against the sim's ground truth.

Renders the MuJoCo scene offline (no ROS), with the board tilted and shifted in its grasp.
Needs mujoco and maze_sim_description (both in the maze-sim container); skipped without them.
"""

from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("maze_sim_description")

from maze_sim_description.camera_math import look_at_quaternion, pinhole_intrinsics, quat_to_matrix  # noqa: E402
from maze_sim_description.config import load_sim_config  # noqa: E402
from maze_sim_description.scene_builder import build_scene  # noqa: E402

from maze_perception.board_pose import Camera, estimate_board_pose  # noqa: E402
from maze_perception.markers import MarkerDetector  # noqa: E402


def _sim_pkg():
    import maze_sim_description

    here = Path(maze_sim_description.__file__).resolve().parents[1]
    if (here / "config" / "sim.yaml").exists():  # source tree
        return here
    from ament_index_python.packages import get_package_share_directory  # installed

    return Path(get_package_share_directory("maze_sim_description"))


@pytest.fixture(scope="module")
def sim():
    pkg = _sim_pkg()
    cfg = load_sim_config(pkg / "config" / "sim.yaml")
    model = mujoco.MjModel.from_xml_string(build_scene(cfg, pkg / "meshes" / "board"))
    w, h = cfg.camera.resolution
    try:
        renderer = mujoco.Renderer(model, h, w)
    except Exception as e:  # no OpenGL context on this machine
        pytest.skip(f"cannot render offscreen: {e}")
    yield cfg, model, renderer
    renderer.close()


def _angle_deg(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


@pytest.mark.parametrize("seed", range(6))
def test_board_pose_matches_ground_truth(sim, cfg, seed):
    sim_cfg, model, renderer = sim
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
    qadr = model.jnt_qposadr[model.body("board").jntadr[0]]
    if seed:  # tilt up to 6 deg about a random horizontal axis, shift up to 2 cm
        rng = np.random.default_rng(seed)
        axis = np.append(rng.normal(size=2), 0.0)
        q = np.zeros(4)
        mujoco.mju_axisAngle2Quat(q, axis / np.linalg.norm(axis), np.radians(rng.uniform(-6, 6)))
        q_new = np.zeros(4)
        mujoco.mju_mulQuat(q_new, q, data.qpos[qadr + 3:qadr + 7].copy())
        data.qpos[qadr + 3:qadr + 7] = q_new
        data.qpos[qadr:qadr + 3] += rng.uniform(-0.02, 0.02, 3)
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera="overhead_camera")
    image = renderer.render()

    w, h = sim_cfg.camera.resolution
    fx, fy, cx, cy = pinhole_intrinsics(w, h, sim_cfg.camera.hfov_deg)
    camera = Camera(K=np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), D=np.zeros(5), width=w, height=h)
    corners = MarkerDetector(cfg.board.dictionary, ids=cfg.board.markers).detect(image)
    assert sorted(corners) == [0, 1, 2, 3]
    pose = estimate_board_pose(corners, cfg.board, camera, cfg.pose)
    assert pose is not None

    # board in world = camera optical frame in world * board in camera
    R_wc = quat_to_matrix(look_at_quaternion(sim_cfg.camera.position, sim_cfg.camera.look_at, sim_cfg.camera.up))
    p_wb = R_wc @ pose.t + np.asarray(sim_cfg.camera.position)
    R_wb = R_wc @ pose.R
    assert np.linalg.norm(p_wb - data.body("board").xpos) < 1e-3
    assert _angle_deg(R_wb, data.body("board").xmat.reshape(3, 3)) < 0.5

    # the ball's center pixel back onto the board: on the ball's plane (z = ball radius when it
    # rests on the floor; here the board was moved under it, so its own height in the board frame)
    ball_w = data.body("ball").xpos
    truth = data.body("board").xmat.reshape(3, 3).T @ (ball_w - data.body("board").xpos)
    px = camera.K @ (R_wc.T @ (ball_w - np.asarray(sim_cfg.camera.position)))
    found = pose.pixel_to_board(px[:2] / px[2], camera, z=truth[2])[0]
    assert np.linalg.norm(found[:2] - truth[:2]) < 1e-3

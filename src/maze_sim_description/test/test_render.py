"""The overhead camera's image: the board must not be clipped, so colour thresholds tuned on it
(HSV ball tracking, ArUco) mean something on the real C922."""

import mujoco
import numpy as np



def _render(model, data, cfg):
    w, h = cfg.camera.resolution
    with mujoco.Renderer(model, height=h, width=w) as r:
        r.update_scene(data, camera="overhead_camera")
        return r.render().astype(float)


def _pixel(model, data, cfg, p_world):
    """Pixel (u, v) of a world point in the overhead camera, MuJoCo's own projection."""
    cam = data.cam("overhead_camera")
    R = cam.xmat.reshape(3, 3)  # columns: camera x (right), y (up), -z (view)
    p = R.T @ (np.asarray(p_world) - cam.xpos)
    w, h = cfg.camera.resolution
    f = (h / 2) / np.tan(np.radians(model.cam_fovy[model.camera("overhead_camera").id]) / 2)
    return int(round(w / 2 + f * p[0] / -p[2])), int(round(h / 2 - f * p[1] / -p[2]))


def test_board_floor_not_clipped(model, data, cfg):
    # bare floor inside the start cell, 15 mm from the ball
    R_b = data.body("board").xmat.reshape(3, 3)
    floor = data.body("board").xpos + R_b @ np.array([cfg.ball.start[0], cfg.ball.start[1] - 0.015, 0.0])
    u, v = _pixel(model, data, cfg, floor)
    rgb = _render(model, data, cfg)[v - 2 : v + 3, u - 2 : u + 3].reshape(-1, 3).mean(axis=0)
    assert rgb.max() < 245, f"board floor clipped: {rgb}"
    r, g, b = rgb
    assert r > g > b and b / r < 0.8, f"board floor not wood-coloured: {rgb}"  # wood is (0.82, 0.72, 0.55)

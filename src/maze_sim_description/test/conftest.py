from pathlib import Path

import pytest

from maze_sim_description.placeholder_board import make_placeholder_board


@pytest.fixture
def placeholder_mm(tmp_path):
    src = tmp_path / "board_src"
    make_placeholder_board(src)
    return src


def wrap_minimal_model(fragment: str, meshdir: Path) -> str:
    """Wrap a board fragment (<asset>…</asset><worldbody>…</worldbody>) in a model."""
    return f'<mujoco><compiler meshdir="{meshdir}"/>{fragment}</mujoco>'


PKG = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def cfg():
    from maze_sim_description.config import load_sim_config

    return load_sim_config(PKG / "config" / "sim.yaml")


@pytest.fixture(scope="session")
def scene_xml(cfg):
    from maze_sim_description.scene_builder import build_scene

    return build_scene(cfg, PKG / "meshes" / "board")


@pytest.fixture(scope="session")
def model(scene_xml):
    import mujoco

    return mujoco.MjModel.from_xml_string(scene_xml)


@pytest.fixture
def data(model):
    """MjData reset to keyframe "start"."""
    import mujoco

    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, model.key("start").id)
    mujoco.mj_forward(model, d)
    return d

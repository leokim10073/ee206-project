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

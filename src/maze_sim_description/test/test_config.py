import re
from pathlib import Path

import pytest
import yaml

from maze_sim_description.config import ConfigError, load_sim_config

PKG = Path(__file__).resolve().parents[1]
DEFAULT = PKG / "config" / "sim.yaml"


def write(tmp_path, d):
    p = tmp_path / "sim.yaml"
    p.write_text(yaml.safe_dump(d))
    return p


def test_default_file_loads():
    cfg = load_sim_config(DEFAULT)
    assert cfg.physics.timestep == 0.002
    assert cfg.camera.resolution == (1280, 720) and cfg.camera.fps == 60
    assert cfg.robot.ur_type == "ur7e" and len(cfg.robot.initial_joints) == 6


@pytest.mark.parametrize(
    "mutate,key",
    [
        (lambda d: d["camera"].pop("position"), "camera.position"),
        (lambda d: d["camera"].update(fps="sixty"), "camera.fps"),
        (lambda d: d["camera"].update(look_at=d["camera"]["position"]), "camera.look_at"),
        (lambda d: d["camera"].update(up=[0, 0, 1], position=[0, 0, 2], look_at=[0, 0, 0]), "camera.up"),
        (lambda d: d["robot"].update(initial_joints=[0.0] * 5), "robot.initial_joints"),
        (lambda d: d["board"].update(units="inch"), "board.units"),
        (lambda d: d["ball"].update(radius=-0.01), "ball.radius"),
        (lambda d: d["physics"].update(timestep=0), "physics.timestep"),
    ],
)
def test_bad_values_name_the_key(tmp_path, mutate, key):
    d = yaml.safe_load(DEFAULT.read_text())
    mutate(d)
    with pytest.raises(ConfigError, match=f"^{re.escape(key)}"):
        load_sim_config(write(tmp_path, d))


def test_unknown_key_rejected(tmp_path):
    d = yaml.safe_load(DEFAULT.read_text())
    d["camera"]["postion"] = [0, 0, 1]
    with pytest.raises(ConfigError, match=r"^camera\.postion"):
        load_sim_config(write(tmp_path, d))

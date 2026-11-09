"""A bad sim.yaml stops the launch with a message naming the key."""

import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory


def test_bad_config_stops_launch(tmp_path):
    default = os.path.join(get_package_share_directory("maze_sim_description"), "config", "sim.yaml")
    cfg = yaml.safe_load(open(default))
    cfg["camera"]["fps"] = "sixty"
    cfg["board"]["mesh_dir"] = "package://maze_sim_description/meshes/board"
    bad = tmp_path / "sim.yaml"
    bad.write_text(yaml.safe_dump(cfg))
    proc = subprocess.run(
        ["ros2", "launch", "maze_sim_bringup", "sim.launch.py", f"config:={bad}", "viewer:=false", "camera:=false"],
        capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode != 0
    assert "camera.fps" in proc.stderr + proc.stdout
    assert "mujoco_ros2_control" not in proc.stdout  # nothing was started

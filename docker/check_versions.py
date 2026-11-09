"""Fail the image build if the pieces the simulation depends on drifted apart.

The scene keyframe is computed with the Python mujoco package and simulated by the plugin's
libmujoco, so the two must be the same release; the reset, camera and actuator handling the
sim relies on were tested against mujoco_ros2_control 0.1.x.

    python3 check_versions.py [vendor_lib_dir] [plugin_version]
"""

import subprocess
import sys
from pathlib import Path

import mujoco

vendor = Path(sys.argv[1] if len(sys.argv) > 1 else "/opt/ros/humble/opt/mujoco_vendor/lib")
libs = sorted(vendor.glob("libmujoco.so.*.*.*"))
if len(libs) != 1:
    sys.exit(f"expected one libmujoco.so.X.Y.Z in {vendor}, found {[p.name for p in libs]}")
lib_version = libs[0].name.removeprefix("libmujoco.so.")
if lib_version != mujoco.__version__:
    sys.exit(f"pip mujoco {mujoco.__version__} != plugin libmujoco {lib_version}: pin mujoco=={lib_version}")

plugin = sys.argv[2] if len(sys.argv) > 2 else subprocess.run(
    ["dpkg-query", "-W", "-f=${Version}", "ros-humble-mujoco-ros2-control"], capture_output=True, text=True, check=True
).stdout
if not plugin.startswith("0.1."):
    sys.exit(f"mujoco_ros2_control {plugin}: the sim was tested on 0.1.x; rerun the tests and update this check")
print(f"mujoco {mujoco.__version__}, mujoco_ros2_control {plugin}: ok")

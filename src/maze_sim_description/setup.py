import os
from glob import glob

from setuptools import find_packages, setup

package_name = "maze_sim_description"


def tree(src):
    """data_files entries that install every file under src, keeping its layout."""
    return [
        (os.path.join("share", package_name, root), [os.path.join(root, f) for f in files])
        for root, _, files in os.walk(src)
        if files
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/meshes/board", glob("meshes/board/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        *tree("mjcf"),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "build_board_assets = maze_sim_description.board_assets:main",
            "make_placeholder_board = maze_sim_description.placeholder_board:main",
            "write_scene = maze_sim_description.scene_builder:main",
        ],
    },
    zip_safe=True,
    maintainer="Leo Chen",
    maintainer_email="leotcchen@berkeley.edu",
    description="MuJoCo scene, board assets and sim.yaml configuration for the maze simulation.",
    license="MIT",
    tests_require=["pytest"],
)

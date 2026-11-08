from glob import glob

from setuptools import find_packages, setup

package_name = "maze_sim_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/meshes/board", glob("meshes/board/*")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "build_board_assets = maze_sim_description.board_assets:main",
            "make_placeholder_board = maze_sim_description.placeholder_board:main",
        ],
    },
    zip_safe=True,
    maintainer="Leo Chen",
    maintainer_email="leotcchen@berkeley.edu",
    description="MuJoCo scene, board assets and sim.yaml configuration for the maze simulation.",
    license="MIT",
    tests_require=["pytest"],
)

from glob import glob

from setuptools import find_packages, setup

package_name = "maze_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "board_tracker = maze_perception.board_node:main",
    ]},
    zip_safe=True,
    maintainer="Zhuoren Li",
    maintainer_email="gabriel.li@berkeley.edu",
    description="Overhead-camera perception for the maze: ArUco board pose, rectified board image.",
    license="MIT",
    tests_require=["pytest"],
)

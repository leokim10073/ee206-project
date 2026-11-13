from glob import glob

from setuptools import find_packages, setup

package_name = "maze_cad_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "cad_map = maze_cad_map.node:main",
    ]},
    zip_safe=True,
    maintainer="Jing Xu",
    maintainer_email="jing_xu1112@berkeley.edu",
    description="Publishes a maze level's MazeMap from the CAD source, as a stand-in for perception P2.",
    license="MIT",
    tests_require=["pytest"],
)

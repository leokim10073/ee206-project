from glob import glob

from setuptools import find_packages, setup

package_name = "maze_sim_bringup"

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
    entry_points={"console_scripts": ["sim_tools = maze_sim_bringup.sim_tools_node:main"]},
    zip_safe=True,
    maintainer="Leo Chen",
    maintainer_email="leotcchen@berkeley.edu",
    description="Launch file, controllers and sim-only services for the maze simulation.",
    license="MIT",
    tests_require=["pytest"],
)

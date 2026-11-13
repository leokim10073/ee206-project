"""The CAD maze map on /maze/map. Run from the repo root (the default cad_scripts is relative to it):
    ros2 launch maze_cad_map cad_map.launch.py               # next to the sim
    ros2 launch maze_cad_map cad_map.launch.py level:=L1 use_sim_time:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true", description="true with the sim, false on the robot"),
        DeclareLaunchArgument("level", default_value="L1", description="key in maze_CAD/scripts/levels.py"),
        DeclareLaunchArgument("cad_scripts", default_value=os.path.join(os.getcwd(), "maze_CAD", "scripts"),
                              description="the repo's maze_CAD/scripts directory"),
        DeclareLaunchArgument("resolution", default_value="0.005", description="m per grid cell"),
    ]
    cad_map = Node(
        package="maze_cad_map",
        executable="cad_map",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "level": LaunchConfiguration("level"),
            "cad_scripts": LaunchConfiguration("cad_scripts"),
            "resolution": ParameterValue(LaunchConfiguration("resolution"), value_type=float),
        }],
    )
    return LaunchDescription([*args, cad_map])

"""Perception nodes. With the sim:   ros2 launch maze_perception perception.launch.py
On the robot (wall clock):          ros2 launch maze_perception perception.launch.py use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true", description="true with the sim, false on the robot"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution(
            [FindPackageShare("maze_perception"), "config", "perception.yaml"])),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("rectified_rate", default_value="5.0", description="Hz of /perception/board_image; 0 = off"),
    ]
    board_tracker = Node(
        package="maze_perception",
        executable="board_tracker",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "config": LaunchConfiguration("config"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rectified_rate": ParameterValue(LaunchConfiguration("rectified_rate"), value_type=float),
        }],
    )
    return LaunchDescription([*args, board_tracker])

"""Start the maze simulation with the real UR7e's ROS interfaces.

    ros2 launch maze_sim_bringup sim.launch.py config:=<sim.yaml> viewer:=true camera:=true

The config is validated first; a bad value ends the launch with a message naming the key.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from maze_sim_description.config import load_sim_config, resolve_path
from maze_sim_description.scene_builder import GRIPPER_ACTUATOR, write_scene

ACTIVE = ["joint_state_broadcaster", "scaled_joint_trajectory_controller", "gripper_controller"]
INACTIVE = ["forward_velocity_controller", "forward_position_controller"]


def _scene_path():
    ros_home = Path(os.environ.get("ROS_HOME", Path.home() / ".ros"))
    return ros_home / "maze_sim" / "scene.xml"


def _setup(context):
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    viewer = LaunchConfiguration("viewer").perform(context).lower() == "true"
    cfg = load_sim_config(config_path)  # ConfigError stops the launch with the key in its message
    board_dir = resolve_path(cfg.board.mesh_dir, config_path.parent)
    scene = write_scene(cfg, board_dir, _scene_path())

    bringup = Path(get_package_share_directory("maze_sim_bringup"))
    description = Path(get_package_share_directory("maze_sim_description"))
    import xacro

    urdf = xacro.process_file(
        str(description / "urdf" / "maze_sim.urdf.xacro"),
        mappings={
            "ur_type": cfg.robot.ur_type,
            "mujoco_model": str(scene),
            "pids_config_file": str(bringup / "config" / "mujoco_pids.yaml"),
            "headless": str(not viewer).lower(),
            "sim_speed_factor": str(cfg.physics.real_time_factor),
            "gripper_joint": GRIPPER_ACTUATOR,
        },
    ).toxml()
    robot_description = {"robot_description": ParameterValue(urdf, value_type=str)}
    controllers = str(bringup / "config" / "controllers.yaml")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            output="both",
            emulate_tty=True,
            parameters=[{"use_sim_time": True}, controllers],
            remappings=[("~/robot_description", "/robot_description")],
            on_exit=Shutdown(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*ACTIVE, "--param-file", controllers],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*INACTIVE, "--inactive", "--param-file", controllers],
            output="both",
        ),
    ]
    return nodes


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("maze_sim_description"), "config", "sim.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config, description="path to sim.yaml"),
            DeclareLaunchArgument("viewer", default_value="true", description="open the MuJoCo viewer"),
            DeclareLaunchArgument("camera", default_value="true", description="publish overhead camera images"),
            OpaqueFunction(function=_setup),
        ]
    )

"""Start the maze simulation with the real UR7e's ROS interfaces.

    ros2 launch maze_sim_bringup sim.launch.py config:=<sim.yaml> viewer:=true camera:=true

The config is validated first; a bad value ends the launch with a message naming the key.
"""

import os
from pathlib import Path

import yaml
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


def _out_dir():
    return Path(os.environ.get("ROS_HOME", Path.home() / ".ros")) / "maze_sim"


def _grid_rate(fps, timestep):
    """The plugin renders on the first step where its ROS clock is at least 1/rate past the last
    frame, so frames land on whole steps: 60 fps at 2 ms steps asked as 60 Hz comes out every
    9 steps (55.6 Hz). Ask for a period half a step shorter than the whole number of steps
    nearest 1/fps from below, so frames come every 8 steps (62.5 Hz)."""
    steps = max(2, int(1.0 / (fps * timestep) + 1e-9))
    return 1.0 / ((steps - 0.5) * timestep)


def _write_plugins(cfg, camera, path):
    """mujoco_ros2_control plugin parameters: board and ball poses always, the camera if asked."""
    plugins = {
        "free_joint_state_publisher": {
            "type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
            "topic": "/sim/free_joint_states",
            "frame_id": "",
            "body_names": ["board", "ball"],
            "publish_rate": 100.0,
        }
    }
    if camera:
        plugins["camera_plugin"] = {
            "type": "mujoco_ros2_control_plugins/CameraPlugin",
            "camera_publish_rate": _grid_rate(cfg.camera.fps, cfg.physics.timestep),
            "overhead_camera": {
                "policy": "streaming",
                "frame_name": "camera_optical_frame",
                "info_topic": "/camera/camera_info",
                "image_topic": "/camera/image_raw",
                "depth_topic": "/sim/camera/depth",
            },
        }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": {"mujoco_plugins": plugins}}}))
    return path


def _setup(context):
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    viewer = LaunchConfiguration("viewer").perform(context).lower() == "true"
    camera = LaunchConfiguration("camera").perform(context).lower() == "true"
    cfg = load_sim_config(config_path)  # ConfigError stops the launch with the key in its message
    board_dir = resolve_path(cfg.board.mesh_dir, config_path.parent)
    scene = write_scene(cfg, board_dir, _out_dir() / "scene.xml")
    plugins = _write_plugins(cfg, camera, _out_dir() / "plugins.yaml")

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
            parameters=[{"use_sim_time": True}, controllers, str(plugins)],
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
        Node(package="maze_sim_nodes", executable="pose_relay", output="both"),
        Node(
            package="maze_sim_bringup",
            executable="sim_tools",
            output="both",
            parameters=[{"use_sim_time": True, "config": str(config_path), "camera": camera}],
        ),
        Node(
            package="maze_sim_bringup",
            executable="gripper",
            output="both",
            parameters=[{"use_sim_time": True}],
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

"""pick_place_demo with its robot-specific parameters from a YAML file (default: config/pick_place_demo.yaml)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur5_robotiq_manipulation"), "config", "pick_place_demo.yaml"]
                ),
                description="Parameter file of the arm (groups, named states, gripper joint, poses)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ur5_robotiq_manipulation",
                executable="pick_place_demo",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
                ],
            ),
        ]
    )

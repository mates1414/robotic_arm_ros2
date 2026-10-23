"""Shows the UR5 + Robotiq 2F-85 model in RViz with joint sliders (no simulation)."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("ur5_robotiq_description"), "urdf", "ur5_robotiq.urdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config]),
        ]
    )

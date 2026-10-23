"""move_group (+ RViz) for the UR5 + Robotiq 2F-85.

Based on ur_moveit_config/launch/ur_moveit.launch.py. The URDF is not loaded here: move_group and RViz take it from
the /robot_description topic of the robot_state_publisher started by the simulation (or a driver).
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur5_robotiq_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur5_robotiq.srdf.xacro", {"name": "ur"})
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("ur5_robotiq_moveit_config"), "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use the simulation clock"
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the warehouse database should be stored",
            ),
            wait_robot_description,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_description,
                    on_exit=[move_group_node, rviz_node],
                )
            ),
        ]
    )

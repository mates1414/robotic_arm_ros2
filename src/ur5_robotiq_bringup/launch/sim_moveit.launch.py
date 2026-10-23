"""Gazebo simulation + MoveIt 2 (move_group and RViz with the MotionPlanning panel)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Scoped, so that launch_rviz:=false of the simulation does not leak into the MoveIt include below
    sim = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ur5_robotiq_bringup"), "launch", "sim.launch.py"])
                ),
                launch_arguments={
                    "launch_rviz": "false",
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                }.items(),
            )
        ]
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur5_robotiq_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start the Gazebo GUI?"),
            sim,
            move_group,
        ]
    )

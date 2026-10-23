"""UR5 + Robotiq 2F-85 in Gazebo Harmonic with ros2_control.

Wraps ur_simulation_gz/launch/ur_sim_control.launch.py (robot_state_publisher, Gazebo, spawn, /clock bridge, arm
controllers) with this project's URDF, controllers and world, and adds the gripper controller and the grasp
attachment bridge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("ur5_robotiq_bringup")

    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur5",
            "description_file": PathJoinSubstitution(
                [FindPackageShare("ur5_robotiq_description"), "urdf", "ur5_robotiq.urdf.xacro"]
            ),
            "controllers_file": PathJoinSubstitution([bringup, "config", "ur5_robotiq_controllers.yaml"]),
            "world_file": LaunchConfiguration("world_file"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "safety_limits": "true",
        }.items(),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # Grasp attachment (DetachableJoint in ur5_robotiq_description) <-> ROS
    grasp_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="grasp_bridge",
        arguments=[
            "/ur5/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/ur5/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/ur5/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        output="screen",
    )

    # The DetachableJoint system starts attached. Once the gripper controller is up the robot is in Gazebo, so detach
    # then; a few repeats cover the bridge's discovery, extra detach requests are ignored.
    detach_at_start = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[
                ExecuteProcess(
                    cmd=["ros2", "topic", "pub", "-w", "1", "-t", "3", "-r", "2",
                         "/ur5/grasp/detach", "std_msgs/msg/Empty", "{}"],
                    output="log",
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start the Gazebo GUI?"),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution([bringup, "worlds", "ur5_workcell.sdf"]),
                description="Gazebo world (absolute path or a name from the gazebosim worlds collection)",
            ),
            ur_sim_control,
            gripper_controller_spawner,
            grasp_bridge,
            detach_at_start,
        ]
    )

# UR5 + Robotiq 2F-85 — ROS 2 Jazzy / Gazebo Harmonic / MoveIt 2

ROS 2 port of [mates1414/robotic_arm](https://github.com/mates1414/robotic_arm) (ROS 1 Noetic, Gazebo Classic,
MoveIt 1). Everything runs in a Docker container on an Ubuntu 24.04 host.

| | ROS 1 repo | this repo |
|---|---|---|
| ROS | Noetic (EOL) | Jazzy (LTS, until 05/2029) |
| Simulator | Gazebo Classic 11 | Gazebo Harmonic |
| Motion planning | MoveIt 1, `moveit_commander` (Python) | MoveIt 2, `MoveGroupInterface` (C++) |
| Grasp in simulation | `gazebo_grasp_fix` | gz-sim `DetachableJoint` |
| UR / Robotiq upstream | vendored source copies | apt binaries, nothing vendored |

## Upstream packages (apt, no fork needed)

| Package | Source | Used for |
|---|---|---|
| `ur_description` | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | UR5 URDF macro, meshes, kinematics |
| `ur_simulation_gz` | [UniversalRobots/Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation) | Gazebo launch, arm ros2_control |
| `ur_robot_driver`, `ur_controllers`, `ur_moveit_config` | [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | controllers, planner configs, the real robot later |
| `robotiq_description` | [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | 2F-85 links and meshes |
| `gz_ros2_control`, `ros_gz`, `moveit` | ros-controls, gazebosim, moveit | |

The released Jazzy `robotiq_description` (0.0.1) still writes its ros2_control block for `ign_ros2_control`, so only
its links are used; the gripper's ros2_control for Harmonic is in `ur5_robotiq_description`. `robotiq_driver` (real
gripper) is not released for Jazzy and has to be built from source when the hardware is used.

## Packages

| Package | Contents |
|---|---|
| `ur5_robotiq_description` | `ur5_robotiq.urdf.xacro`: UR5 + 2F-85 + `grasp_link` TCP + gripper ros2_control + grasp attachment |
| `ur5_robotiq_moveit_config` | SRDF (`manipulator`, `gripper`; `home`, `up`, `open`, `closed`), MoveIt controllers, `move_group.launch.py` |
| `ur5_robotiq_bringup` | `sim.launch.py`, `sim_moveit.launch.py`, controllers, `worlds/ur5_workcell.sdf` (floor + 5 cm cube) |
| `ur5_robotiq_manipulation` | `pick_place_demo` (C++, MoveGroupInterface) |

## Quick start

```bash
./docker/run.sh --build     # once: builds the ur5_ros2:jazzy image (~7 GB)
./docker/run.sh             # shell in the container (started on demand), at the workspace root

# in the container
colcon build --symlink-install
source install/setup.bash
ros2 launch ur5_robotiq_bringup sim_moveit.launch.py          # Gazebo + MoveIt + RViz
# second shell (./docker/run.sh):
ros2 launch ur5_robotiq_manipulation pick_place_demo.launch.py
```

The demo moves the cube from (0.5, 0) to (0.4, -0.3). Its parameters are in
`ur5_robotiq_manipulation/config/pick_place_demo.yaml`: the robot-specific names (`arm_group`, `gripper_group`,
`arm_home_state`, `gripper_open_state`, `gripper_joint`, `attach_topic`, `detach_topic`) and the poses (`pick_x`,
`pick_y`, `place_x`, `place_y`, `grasp_z`, `approach_z`, `grasp_position`). For another arm copy the file and pass
`config_file:=<path>`. Without the launch file the node falls back to the UR5 + 2F-85 values
(`ros2 run ur5_robotiq_manipulation pick_place_demo --ros-args -p use_sim_time:=true`).

Other entry points:

```bash
ros2 launch ur5_robotiq_bringup sim.launch.py                 # Gazebo + controllers + RViz, no MoveIt
ros2 launch ur5_robotiq_description view.launch.py            # model only, joint sliders
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5   # bare upstream UR5, no gripper
```

## Docker

`docker/run.sh` keeps one container (`ur5_ros2_dev`) running and `exec`s into it. The repository is mounted at the
same path, the user has the host UID/GID, X11 is forwarded and the NVIDIA GPU is passed in when
nvidia-container-toolkit is installed. Host networking with `ROS_DOMAIN_ID=21` (override with `ROS_DOMAIN_ID=...
./docker/run.sh`) so the container does not talk to other ROS containers on domain 0. RMW is Cyclone DDS.

`./docker/run.sh <command>` runs a command in the container with the environment loaded, `./docker/run.sh --stop`
removes the container.

## Gripper and grasping in Gazebo

* Only `robotiq_85_left_knuckle_joint` is commanded (`gripper_controller`, `control_msgs/action/GripperCommand` on
  `/gripper_controller/gripper_cmd`, 0.0 open … 0.8 closed). The five other joints follow it through their URDF
  `<mimic>` tags, driven by gz_ros2_control. Gazebo prints that DART has no mimic constraints; that is expected.
* Friction alone does not hold objects reliably (the problem in the ROS 1 repo's `grasp.md`). The URDF loads gz-sim's
  `DetachableJoint` system: `std_msgs/Empty` on `/ur5/grasp/attach` fixes the model named by the xacro argument
  `grasp_object` (default `cube`) to the wrist at its current pose, `/ur5/grasp/detach` releases it, `/ur5/grasp/state`
  reports it. The system starts attached; `sim.launch.py` detaches it at start-up.
* Close to the object's width (about 0.35 rad for 5 cm), not fully: while attached the fingers do not collide with the
  object, so a full close drives them into it and the object sticks to the fingers after the release. Open first,
  then detach.

## Known pitfalls

* `ur_simulation_gz` parses the generated URDF as YAML: comments in the xacro files must not contain a colon followed
  by a space.
* Launch arguments of an `IncludeLaunchDescription` leak into the including file; `sim_moveit.launch.py` scopes the
  simulation include in a `GroupAction`.

## Next steps (from the ROS 1 repo)

1. Wrist camera (gz `camera` sensor + `ros_gz_image` bridge) and an AprilTag cube in the world.
2. `apriltag_ros` (already in the image) for the tag pose.
3. Pick-and-place node driven by the detected pose; the cube as a MoveIt collision object attached during transport.
4. Real UR5 (CB3, PolyScope ≥ 3.14.3) with `ur_robot_driver` and the Robotiq driver.

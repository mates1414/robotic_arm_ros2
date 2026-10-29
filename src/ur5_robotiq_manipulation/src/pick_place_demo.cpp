// Pick-and-place demo with MoveGroupInterface: picks the cube of the ur5_workcell world from a fixed pose
// (no perception yet) and puts it down at another spot. In Gazebo the grasped cube is fixed to the gripper through
// /ur5/grasp/attach and released through /ur5/grasp/detach.
//
// Robot-specific names (planning groups, named states, gripper joint, grasp topics) are parameters, see
// config/pick_place_demo.yaml; another arm only needs its own copy of that file.
//
//   ros2 launch ur5_robotiq_manipulation pick_place_demo.launch.py

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace
{
using moveit::planning_interface::MoveGroupInterface;

const rclcpp::Logger kLogger = rclcpp::get_logger("pick_place_demo");

// grasp_link pointing straight down (180 deg about the world x axis)
geometry_msgs::msg::Pose top_down_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 1.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

bool plan_and_execute(MoveGroupInterface & group, const std::string & what)
{
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(kLogger, "planning failed: %s", what.c_str());
    return false;
  }
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(kLogger, "execution failed: %s", what.c_str());
    return false;
  }
  RCLCPP_INFO(kLogger, "done: %s", what.c_str());
  return true;
}

bool go_named(MoveGroupInterface & group, const std::string & state)
{
  group.setNamedTarget(state);
  return plan_and_execute(group, group.getName() + " -> " + state);
}

// Closes to the object's width rather than fully: once the object is attached the fingers no longer collide with it,
// so a full close would drive them into the object.
bool close_gripper(MoveGroupInterface & gripper, const std::string & joint, double position)
{
  gripper.setJointValueTarget(joint, position);
  return plan_and_execute(gripper, "gripper -> " + joint + " = " + std::to_string(position));
}

bool go_pose(MoveGroupInterface & arm, const geometry_msgs::msg::Pose & pose, const std::string & what)
{
  arm.setPoseTarget(pose);
  return plan_and_execute(arm, what);
}

// Straight-line motion of grasp_link, e.g. the final approach and the lift
bool go_straight(MoveGroupInterface & arm, const geometry_msgs::msg::Pose & target, const std::string & what)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = arm.computeCartesianPath({target}, 0.005, trajectory);
  if (fraction < 0.99) {
    RCLCPP_ERROR(kLogger, "cartesian path %s: only %.0f %% feasible", what.c_str(), fraction * 100.0);
    return false;
  }
  if (arm.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(kLogger, "execution failed: %s", what.c_str());
    return false;
  }
  RCLCPP_INFO(kLogger, "done: %s", what.c_str());
  return true;
}

// Fixes the object to / releases it from the gripper in Gazebo (DetachableJoint, see ur5_robotiq_description)
bool send_grasp_command(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & pub, const std::string & what)
{
  for (int i = 0; i < 20 && pub->get_subscription_count() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (pub->get_subscription_count() == 0) {
    RCLCPP_ERROR(kLogger, "nobody listens on %s (is the simulation running?)", pub->get_topic_name());
    return false;
  }
  pub->publish(std_msgs::msg::Empty());
  // the attachment is made in the next physics steps
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(kLogger, "done: %s", what.c_str());
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pick_place_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  const auto param = [&node](const std::string & name, double fallback) {
      return node->has_parameter(name) ? node->get_parameter(name).as_double() : fallback;
    };
  const auto string_param = [&node](const std::string & name, const std::string & fallback) {
      return node->has_parameter(name) ? node->get_parameter(name).as_string() : fallback;
    };

  // robot-specific names, the fallbacks are the UR5 + Robotiq 2F-85 ones of ur5_robotiq_moveit_config
  const std::string arm_group = string_param("arm_group", "manipulator");
  const std::string gripper_group = string_param("gripper_group", "gripper");
  const std::string arm_home_state = string_param("arm_home_state", "home");
  const std::string gripper_open_state = string_param("gripper_open_state", "open");
  const std::string gripper_joint = string_param("gripper_joint", "robotiq_85_left_knuckle_joint");
  const std::string attach_topic = string_param("attach_topic", "/ur5/grasp/attach");
  const std::string detach_topic = string_param("detach_topic", "/ur5/grasp/detach");

  const double pick_x = param("pick_x", 0.5);
  const double pick_y = param("pick_y", 0.0);
  const double place_x = param("place_x", 0.4);
  const double place_y = param("place_y", -0.3);
  const double grasp_z = param("grasp_z", 0.03);       // grasp_link height at the grasp (cube center 0.025)
  const double approach_z = param("approach_z", 0.20);  // height of the approach / retreat poses
  // gripper_joint position at the grasp; for the 2F-85 knuckle ~0.085 m * (1 - q / 0.8) between the pads,
  // so 0.35 rad squeezes the 5 cm cube slightly
  const double grasp_position = param("grasp_position", 0.35);

  const auto attach_pub = node->create_publisher<std_msgs::msg::Empty>(attach_topic, 1);
  const auto detach_pub = node->create_publisher<std_msgs::msg::Empty>(detach_topic, 1);

  MoveGroupInterface arm(node, arm_group);
  MoveGroupInterface gripper(node, gripper_group);
  arm.setPlanningTime(5.0);
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  RCLCPP_INFO(
    kLogger, "planning frame %s, end effector %s", arm.getPlanningFrame().c_str(),
    arm.getEndEffectorLink().c_str());

  const bool ok =
    go_named(arm, arm_home_state) &&
    go_named(gripper, gripper_open_state) &&
    go_pose(arm, top_down_pose(pick_x, pick_y, approach_z), "pre-grasp") &&
    go_straight(arm, top_down_pose(pick_x, pick_y, grasp_z), "approach") &&
    close_gripper(gripper, gripper_joint, grasp_position) &&
    send_grasp_command(attach_pub, "attach") &&
    go_straight(arm, top_down_pose(pick_x, pick_y, approach_z), "lift") &&
    go_pose(arm, top_down_pose(place_x, place_y, approach_z), "pre-place") &&
    go_straight(arm, top_down_pose(place_x, place_y, grasp_z + 0.005), "lower") &&
    go_named(gripper, gripper_open_state) &&
    send_grasp_command(detach_pub, "detach") &&
    go_straight(arm, top_down_pose(place_x, place_y, approach_z), "retreat") &&
    go_named(arm, arm_home_state);

  RCLCPP_INFO(kLogger, ok ? "pick and place finished" : "pick and place aborted");

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}

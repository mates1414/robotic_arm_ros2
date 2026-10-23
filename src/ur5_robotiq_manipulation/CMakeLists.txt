cmake_minimum_required(VERSION 3.22)
project(ur5_robotiq_manipulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_compile_features(pick_place_demo PUBLIC cxx_std_17)
ament_target_dependencies(pick_place_demo geometry_msgs moveit_ros_planning_interface rclcpp std_msgs)

install(TARGETS pick_place_demo DESTINATION lib/${PROJECT_NAME})

ament_package()

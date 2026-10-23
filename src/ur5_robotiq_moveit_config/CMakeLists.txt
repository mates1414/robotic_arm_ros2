cmake_minimum_required(VERSION 3.22)
project(ur5_robotiq_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

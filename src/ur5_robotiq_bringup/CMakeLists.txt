cmake_minimum_required(VERSION 3.22)
project(ur5_robotiq_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

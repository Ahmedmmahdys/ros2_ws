cmake_minimum_required(VERSION 3.5)
project(moveit_improved_interface)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()

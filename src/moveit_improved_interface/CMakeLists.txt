cmake_minimum_required(VERSION 3.5)
project(moveit_improved_interface)

find_package(ament_cmake REQUIRED)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT OR DEFINED CATKIN_SYMLINK_INSTALL)
  message(STATUS "Ignoring catkin-specific variables for ${PROJECT_NAME} on ROS 2")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()

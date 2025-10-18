cmake_minimum_required(VERSION 3.8)
project(crane_interfaces)

option(CATKIN_INSTALL_INTO_PREFIX_ROOT
  "Compatibility placeholder for catkin workspaces. Ignored in ROS 2 builds." OFF)
option(CATKIN_SYMLINK_INSTALL
  "Compatibility placeholder for catkin workspaces. Ignored in ROS 2 builds." OFF)
mark_as_advanced(CATKIN_INSTALL_INTO_PREFIX_ROOT CATKIN_SYMLINK_INSTALL)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/PanelTask.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

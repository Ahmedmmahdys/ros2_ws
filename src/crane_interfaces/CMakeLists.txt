cmake_minimum_required(VERSION 3.8)
project(crane_interfaces)

# Catkin is a ROS 1 build system. A handful of users have reported that these
# cache variables linger in their environment after migrating from ROS 1, which
# causes CMake to warn that the variables are unused. Clear them proactively so
# the build runs cleanly under ROS 2 without surfacing catkin terminology.
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
endif()

if(DEFINED CATKIN_SYMLINK_INSTALL)
  unset(CATKIN_SYMLINK_INSTALL CACHE)
  unset(CATKIN_SYMLINK_INSTALL)
endif()

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

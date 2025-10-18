cmake_minimum_required(VERSION 3.8)
project(crane_interfaces)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  message(STATUS "CATKIN_INSTALL_INTO_PREFIX_ROOT=${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
endif()

if(DEFINED CATKIN_SYMLINK_INSTALL)
  message(STATUS "CATKIN_SYMLINK_INSTALL=${CATKIN_SYMLINK_INSTALL}")
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

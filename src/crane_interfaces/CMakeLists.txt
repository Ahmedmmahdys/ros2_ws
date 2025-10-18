cmake_minimum_required(VERSION 3.8)
project(crane_interfaces)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_catkin_install_into_prefix_root "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT "${_catkin_install_into_prefix_root}" CACHE INTERNAL "Ignored catkin build option" FORCE)
  unset(_catkin_install_into_prefix_root)
endif()

if(DEFINED CATKIN_SYMLINK_INSTALL)
  set(_catkin_symlink_install "${CATKIN_SYMLINK_INSTALL}")
  unset(CATKIN_SYMLINK_INSTALL CACHE)
  set(CATKIN_SYMLINK_INSTALL "${_catkin_symlink_install}" CACHE INTERNAL "Ignored catkin build option" FORCE)
  unset(_catkin_symlink_install)
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

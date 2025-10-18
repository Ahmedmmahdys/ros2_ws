cmake_minimum_required(VERSION 3.8)
project(rcan_executor)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  message(STATUS "CATKIN_INSTALL_INTO_PREFIX_ROOT=${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
endif()

if(DEFINED CATKIN_SYMLINK_INSTALL)
  message(STATUS "CATKIN_SYMLINK_INSTALL=${CATKIN_SYMLINK_INSTALL}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(crane_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()

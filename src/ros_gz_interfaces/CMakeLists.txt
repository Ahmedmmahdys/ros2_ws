cmake_minimum_required(VERSION 3.5)
project(ros_gz_interfaces)

find_package(ament_cmake REQUIRED)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  unset(CATKIN_SYMLINK_INSTALL CACHE)
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()

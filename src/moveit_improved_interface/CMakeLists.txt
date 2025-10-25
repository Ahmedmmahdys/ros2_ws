cmake_minimum_required(VERSION 3.5)
project(moveit_improved_interface)

find_package(ament_cmake REQUIRED)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  unset(CATKIN_SYMLINK_INSTALL CACHE)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh)
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.bat)

ament_package()

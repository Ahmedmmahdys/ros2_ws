cmake_minimum_required(VERSION 3.8)
project(crane_interfaces)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(
    CATKIN_INSTALL_INTO_PREFIX_ROOT
    "${CATKIN_INSTALL_INTO_PREFIX_ROOT}"
    CACHE INTERNAL "Consumed for compatibility with catkin builds"
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/PanelTask.msg"
  "msg/TaskStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(multi_robot_sim)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(ros_gz_interfaces REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8      REQUIRED)
find_package(ignition-math6      REQUIRED)
find_package(ignition-plugin1     REQUIRED)
find_package(ignition-gazebo6    REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


ament_export_dependencies(rclcpp_action)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY launch meshes models urdf worlds config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

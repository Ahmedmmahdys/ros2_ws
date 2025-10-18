cmake_minimum_required(VERSION 3.8)
project(rcan_executor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(crane_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()

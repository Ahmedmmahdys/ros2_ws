cmake_minimum_required(VERSION 3.5)
project(ros_gz_interfaces)

find_package(ament_cmake REQUIRED)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()

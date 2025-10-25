cmake_minimum_required(VERSION 3.5)
project(control_msgs)

find_package(ament_cmake REQUIRED)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()

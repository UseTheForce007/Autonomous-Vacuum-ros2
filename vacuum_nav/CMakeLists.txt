cmake_minimum_required(VERSION 3.8)
project(vacuum_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()

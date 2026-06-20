cmake_minimum_required(VERSION 3.10)
project(vacuum_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()

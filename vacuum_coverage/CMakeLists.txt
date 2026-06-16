cmake_minimum_required(VERSION 3.10)
project(vacuum_coverage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(library_sources
  src/graph.cpp
  src/map_to_graph.cpp
)

add_library(${PROJECT_NAME} ${library_sources})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(map_to_graph_node src/map_to_graph_node.cpp)
target_link_libraries(map_to_graph_node ${PROJECT_NAME})
ament_target_dependencies(map_to_graph_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  map_to_graph_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

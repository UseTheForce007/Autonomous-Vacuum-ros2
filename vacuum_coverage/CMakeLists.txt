cmake_minimum_required(VERSION 3.10)
project(vacuum_coverage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(library_sources
  src/graph.cpp
  src/map_to_graph.cpp
  src/stc_planner.cpp
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

add_executable(stc_planner_node src/stc_planner_node.cpp)
target_link_libraries(stc_planner_node ${PROJECT_NAME})
ament_target_dependencies(stc_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
)

add_executable(boustrophedon_planner_node src/boustrophedon_planner_node.cpp)
target_link_libraries(boustrophedon_planner_node ${PROJECT_NAME})
ament_target_dependencies(boustrophedon_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
)

add_executable(coverage_executor_node src/coverage_executor_node.cpp)
target_link_libraries(coverage_executor_node ${PROJECT_NAME})
ament_target_dependencies(coverage_executor_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
)

install(TARGETS
  ${PROJECT_NAME}
  map_to_graph_node
  stc_planner_node
  boustrophedon_planner_node
  coverage_executor_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

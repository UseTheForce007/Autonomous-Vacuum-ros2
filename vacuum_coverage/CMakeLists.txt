cmake_minimum_required(VERSION 3.10)
project(vacuum_coverage)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# --- Nav2 plugin: boustrophedon coverage planner ---
add_library(boustrophedon_coverage_planner SHARED
  src/boustrophedon_coverage_planner.cpp
)

target_include_directories(boustrophedon_coverage_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(boustrophedon_coverage_planner
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  tf2_ros
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core boustrophedon_coverage_plugin.xml)

# --- Coverage executor (bridges path topic → FollowWaypoints) ---
add_executable(coverage_executor_node src/coverage_executor_node.cpp)

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
  boustrophedon_coverage_planner
  coverage_executor_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/extract_poly.py
  scripts/send_coverage_goal.py
  scripts/publish_coverage_path.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

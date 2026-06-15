# Autonomous-Vacuum-ros2

Develop a completely functional pipeline in ROS 2 that autonomously moves the mobile robot to where it needs to be. It must be an optimised version capable of being run on constrained compute (RPI 4).

## Launching env

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Launching nav2

```
ros2 launch vacuum_nav nav2_bringup.launch.py slam:=True
```

- The map is built in memory and published to `/map` (visible in RViz), but it is not automatically saved to disk. You need to trigger the save yourself. After you've driven the robot around enough to build a good map, run:

```
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Or via service call:

```
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map}"
```

## Explorating Frontiers

```
ros2 launch explore_lite explore.launch.py
```

Explore is a frontier-based exploration node (explore_lite package). It drives a robot to autonomously explore unknown space.
Inputs:
- costmap (nav_msgs/OccupancyGrid) — full occupancy grid
- costmap_updates (map_msgs/OccupancyGridUpdate) — incremental map updates
- explore/resume (std_msgs/Bool) — start/stop exploration
- TF: base_link → map for robot pose
- Parameters: planner_frequency, progress_timeout, potential_scale, gain_scale, min_frontier_size, visualize, return_to_init, etc.
Outputs:
- Action client → navigate_to_pose (sends frontier centroid goals)
- explore/frontiers (visualization_msgs/MarkerArray) — frontier visualizations (if enabled)
- explore/status (explore_lite_msgs/ExploreStatus) — state machine status (exploration_started, in_progress, paused, complete, returning_to_origin, etc.)
Core algorithm: BFS from the robot's position through free space to find frontier cells (unknown cells adjacent to free space), clusters them, ranks by cost (distance vs. size), and sends the best un-blacklisted frontier as a navigation goal. Monitors progress and blacklists stalled frontiers.

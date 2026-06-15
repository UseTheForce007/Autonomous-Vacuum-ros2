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

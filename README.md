# Autonomous-Vacuum-ros2

## Dependencies

Install the required TurtleBot3 packages:

```bash
sudo apt install ros-jazzy-turtlebot3-gazebo
```


## Launching env

```
ros2 launch vacuum_gz dual_room.launch.py
```

## Launching nav2

```
ros2 launch vacuum_coverage vacuum.launch.py slam:=True
```

- The map is built in memory and published to `/map` (visible in RViz), but it is not automatically saved to disk. You need to trigger the save yourself. After you've driven the robot around enough to build a good map, run:

```
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Or via service call:

```
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map}"
```

## Launching coverage

- First we launch the robot in Localisation mode without slam and feed in the map we have created.

```
ros2 launch vacuum_coverage vacuum.launch.py slam:=False map:=path/to/map_2.0.yaml

```
- Launch the coverage services to plan and execute.

```bash
# Step 1 — compute the coverage path
ros2 service call /planner_server/replan std_srvs/srv/Trigger

# Step 2 — execute it
ros2 service call /start_coverage std_srvs/srv/Trigger
```


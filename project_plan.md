# Project plan: Autonomous Vaccum Cleaner

## Overview

Develop a completely functional pipeline in ROS 2 that autonomously moves the mobile robot to where it needs to be. It must be an optimised version capable of being run on constrained compute (RPI 4).

## System Architecture

### Hardware platform
- Turtlebot3 base with the following sensors attached:
    - LIDAR
    - Optical Camera
    - Wheel Encoder
    - Ultrasonic/IR sensor
- Single CPU core, 1GB RAM

### Software Stack
- C++
- Ubuntu 24
- ROS2 - Jazzy
- Gazebo - Ignition

## Milestones

### Phase 1: Simulation setup

- Robot:
    - Add turtlebot3 and attach sensors
- Environments:
    - Multiple shaped rooms
    - Static obstacles
    - Dynamic obstacles

### Phase 2: NAV2 setup (Not Optimised)

- Implement a basic functionality with a default nav2 setup.
    - Pipe in all the data as is from sensors.
- Add a custom coverage algorithm and connect with nav2.
- Add a docking server.

### Phase 3: Optimisation (full cpu compute)

- The nodes will be profiled at each change and positive changes will be incorporated.
- LIDAR
    - Voxel Downsampling
    - Point cloud to laserscan conversion
- NAV2
    - Detect and improve bottlenecks in all the servers running.
- ROS2
    - Try different DDS profiles to find the lightest and practical one.

### Phase 4: Final deliverable (Limited resource compute)

- Add SIMD and parallelism if possible
- Attempt to run on a single CPU core and 1 GB RAM.


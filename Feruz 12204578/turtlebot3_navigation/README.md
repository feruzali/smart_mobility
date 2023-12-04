# TurtleBot3 Navigation Module

## Author
Feruz 12204578

## Overview
This ROS (Robot Operating System) node facilitates the navigation capabilities of the TurtleBot3 robot. It employs the ROS Navigation Stack to enable autonomous navigation, obstacle avoidance, and path planning.

## ROS Packages Used
- **turtlebot3_navigation**: TurtleBot3-specific ROS package for navigation.
- **robot_localization**: Package for sensor fusion, allowing the integration of sensor data for accurate robot localization.
- **move_base**: ROS package for path planning and navigation execution.
- **teleop_twist_keyboard**: Package for manual robot control using keyboard inputs.

## Node Functionality
1. Utilizes sensor data (Lidar, IMU) for accurate robot localization.
2. Interfaces with the move_base package for path planning and navigation.
3. Implements obstacle avoidance strategies during autonomous navigation.
4. Supports manual control using the teleop_twist_keyboard package.

## ROS Parameters
- **odom_topic**: Topic for odometry data.
- **map_frame**: Frame id of the map.
- **base_frame**: Frame id of the robot base.
- **global_costmap**: Parameters for the global costmap.
- **local_costmap**: Parameters for the local costmap.
- **DWAPlannerROS**: Parameters for the Dynamic Window Approach planner.
- **move_base_simple/goal**: Topic for setting navigation goals.

## Launch Files
- **turtlebot3_navigation.launch**: Launch file to start the navigation module.

## Teleoperation
To teleoperate the robot manually using the keyboard, run:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Running the Navigation Module
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
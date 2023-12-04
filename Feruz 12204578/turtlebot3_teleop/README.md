# TurtleBot3 Teleoperation Module

## Author
Feruz 12204578

## Overview
This Python script enables teleoperation of a TurtleBot3 robot through keyboard input. It publishes Twist messages to the 'cmd_vel' topic, allowing control of linear and angular velocities. The script supports dynamic adjustments of velocity and provides a user-friendly interface.

## ROS Packages Used
- **geometry_msgs**: Standard ROS messages for geometric data.

## Keyboard Controls
- **w/x**: Increase/Decrease linear velocity
- **a/d**: Increase/Decrease angular velocity
- **space key, s**: Force stop
- **CTRL-C**: Quit the teleoperation script

## Velocity Limits
- **Linear**: ~ 0.22
- **Angular**: ~ 2.84

## Operating System Compatibility
This script is designed to work on both Windows (nt) and Linux-based systems.

## Usage
1. Execute the teleoperation script.
   ```bash
   rosrun turtlebot3_teleop_key turtlebot3_teleop_key.py
   ```
2. Use the specified keys to control the robot's linear and angular velocities.
3. Press CTRL-C to quit the teleoperation script.

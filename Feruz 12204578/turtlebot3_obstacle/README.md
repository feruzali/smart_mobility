# TurtleBot3 Obstacle Avoidance Module

## Author
Feruz 12204578

## Overview
This ROS (Robot Operating System) module enables obstacle avoidance for the TurtleBot3 robot through a client-server architecture. The module comprises both client and server components, where the client requests obstacle information from the server, and the server utilizes Lidar sensor data to detect obstacles and dynamically adjusts the robot's path to avoid collisions.

## ROS Packages Used
- **turtlebot3_obstacle**: Custom ROS package for obstacle avoidance.
- **turtlebot3_navigation**: TurtleBot3-specific ROS package for navigation.
- **move_base**: ROS package for path planning and navigation execution.
- **sensor_msgs**: Standard ROS messages for sensor data.
- **geometry_msgs**: Standard ROS messages for geometric data.

## Node Functionality

### Server
1. Utilizes Lidar sensor data for real-time obstacle detection.
2. Interacts with the move_base package to dynamically adjust the robot's path for obstacle avoidance.
3. Publishes obstacle information for visualization and analysis.

### Client
1. Requests obstacle information from the server.
2. Provides an interface for external systems to query the current obstacle status.

## ROS Parameters

### Server
- **lidar_topic**: Topic for Lidar sensor data.
- **obstacle_distance_threshold**: Distance threshold for considering an obstacle.
- **obstacle_avoidance_topic**: Topic for publishing obstacle information.

### Client
- **server_obstacle_info_service**: Service for requesting obstacle information from the server.

## Launch Files
- **turtlebot3_obstacle_server.launch**: Launch file to start the obstacle avoidance server.
- **turtlebot3_obstacle_client.launch**: Launch file to start the obstacle avoidance client.

## Running the Obstacle Avoidance Module

### Server
```bash
roslaunch turtlebot3_obstacle turtlebot3_obstacle_server.launch
```

### Client
```bash
roslaunch turtlebot3_obstacle turtlebot3_obstacle_client.launch
```
# TurtleBot3 SLAM Function

## Overview

This repository contains the source code for a SLAM (Simultaneous Localization and Mapping) function implemented on TurtleBot3 using ROS (Robot Operating System). The SLAM function is specifically designed to operate in a flat world and incorporates IMU (Inertial Measurement Unit) data for accurate mapping.

## Features

- Real-time SLAM using TurtleBot3
- Utilizes IMU data to adjust linear acceleration in the flat world
- Ensures accurate mapping in a gravity-aligned frame

## Prerequisites

Before running the SLAM function, make sure you have the following installed:

- ROS (Robot Operating System)
- TurtleBot3 packages
- Any additional dependencies specified in the package documentation


## Usage

Launch the SLAM node:

    ```bash
    roslaunch turtlebot3_slam turtlebot3_flat_world_imu.launch
    ```


## References

- [ROS Wiki](http://wiki.ros.org/)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Flat World IMU](https://examplelinktoyourdocumentation.com)


## Author

- Abror Khasanov (12204542)
- Contact: abbossxon95@gmail.com


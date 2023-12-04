# TurtleBot3 And PC Setup Report

## Introduction: 

This report provides detailed instructions for setting up TurtleBot3 on a Remote PC. It covers the installation process of Ubuntu 22.03 LTS, ROS2 Humble Hawksbill, and essential ROS packages, ensuring a smooth deployment for controlling TurtleBot3.

## Compatibility Warnings:

### PC Setup Warning:
The instructions in this chapter pertain to the Remote PC, not TurtleBot3. Do not apply these instructions to TurtleBot3 directly.

### Jetson Nano Compatibility:
Jetson Nano does not support Ubuntu 20.04 and later.

## Ubuntu Installation:

1. Download the [Ubuntu 22.04 LTS Desktop image (64-bit)](https://ubuntu.com/download/desktop) for your PC.
2. Follow the provided instructions to install Ubuntu on the PC.

## ROS 2 Installation:

Refer to the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html) for ROS2 Humble installation on the Remote PC.

For most Linux users, it is strongly recommended to use the Debian package installation method.

## Dependent ROS 2 Packages Installation:

1. Open the terminal on the Remote PC with Ctrl+Alt+T.
2. Install Gazebo:
   ```
   $ sudo apt install ros-humble-gazebo-*
   ```
3. Install Cartographer:
   ```
   $ sudo apt install ros-humble-cartographer
   $ sudo apt install ros-humble-cartographer-ros
   ```
4. Install Navigation2:
   ```
   $ sudo apt install ros-humble-navigation2
   $ sudo apt install ros-humble-nav2-bringup
   ```

## TurtleBot3 Packages Installation:
### Install TurtleBot3 via Debian Packages:
```
$ source ~/.bashrc
$ sudo apt install ros-humble-dynamixel-sdk
$ sudo apt install ros-humble-turtlebot3-msgs
$ sudo apt install ros-humble-turtlebot3
```
## Environment Configuration:
Set the ROS environment for the PC:
```
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc
```
## Conclusion

This report outlines a step-by-step guide for configuring a Remote PC to control TurtleBot3. Following these instructions ensures a proper setup, leveraging Ubuntu 22.03 LTS, ROS2 Humble Hawksbill, and essential ROS packages.

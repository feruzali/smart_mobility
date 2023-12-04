#!/bin/bash

#Open the terminal with Ctrl+Alt+T from Remote PC.
#Install Gazebo

sudo apt install ros-humble-gazebo-*

#Install Cartographer

sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

#Install Navigation2

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

#Install TurtleBot3 via Debian Packages.

source ~/.bashrc
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3

#In case you need to build the TurtleBot3 packages with source code, please use the commands below.
#Building the source code provides most up to date contents which may have resolved known issues.
#Make sure to remove the binary packages to avoid redundancy.
#sudo apt remove ros-humble-turtlebot3-msgs
#sudo apt remove ros-humble-turtlebot3
#mkdir -p ~/turtlebot3_ws/src
#cd ~/turtlebot3_ws/src/
#git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
#git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
#git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
#cd ~/turtlebot3_ws
#colcon build --symlink-install
#echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
#source ~/.bashrc

#Set the ROS environment for PC

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

#If you have installed TurtleBot3 using Debian packages with apt install command, you can ignore the warning below.
#bash: /home/{$YOUR_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory
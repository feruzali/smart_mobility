#!/bin/bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch TurtleBot3 Bringup
ros2 launch turtlebot3_bringup robot.launch.py &

# Wait for the TurtleBot3 Bringup to start
sleep 5

# Launch TurtleBot3 Navigation2 with a specified map
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml &

# Wait for the TurtleBot3 Navigation2 to start
sleep 5

# Launch TurtleBot3 teleoperation
ros2 run turtlebot3_teleop teleop_keyboard

# When teleoperation ends, terminate the launched nodes
pkill -f 'ros2 launch turtlebot3_bringup'
pkill -f 'ros2 launch turtlebot3_navigation2'

###### Run in a new terminal ######

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch TurtleBot3 Bringup
ros2 launch turtlebot3_bringup robot.launch.py &

# Wait for the TurtleBot3 Bringup to start
sleep 5

# Launch TurtleBot3 Navigation2 with a specified map
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml &

# Wait for the TurtleBot3 Navigation2 to start
sleep 5

# Launch TurtleBot3 teleoperation
ros2 run turtlebot3_teleop teleop_keyboard

# When teleoperation ends, terminate the launched nodes
pkill -f 'ros2 launch turtlebot3_bringup'
pkill -f 'ros2 launch turtlebot3_navigation2'

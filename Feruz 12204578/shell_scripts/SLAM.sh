roscore
#connecting Rasberry PI and IP address
ssh turtle@{IP_ADDRESS_OF_RASPBERRY_PI}
roslaunch turtlebot3_bringup turtlebot3_robot.launch

#Launching the SLAM
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch

#controlling the Turtlebot3
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

#save the map
rosrun map_server map_saver -f ~/map
          Turtlebot Low Battery Package

                          Overview:
The Turtlebot Low Battery package contains ROS nodes for simulating low battery alerts and monitoring battery levels of a Turtlebot robot.

                            Nodes:
publisher_node.py: Simulates a battery level publisher for the Turtlebot.

subscriber_node.py: Subscribes to battery level messages and triggers alerts for low battery conditions.

                                  Installation

Clone the Repository
git clone https://github.com/your_username/turtlebot_low_battery.git

cd turtlebot_low_battery
Install Dependencies:
Make sure you have ROS installed. Additionally, install any missing dependencies using rosdep.

Build the Package:
catkin_make 
source devel/setup.bash

                                  Usage

Running the Nodes

Start ROS Master:
roscore

Launch Nodes using CLI:
To run the publisher node with a specified rate:
./cli.py publisher --rate 2  # Run publisher at 2 Hz

To run the subscriber node:
./cli.py subscriber  # Run the subscriber node

Viewing Output:
Monitor the published battery levels: rostopic echo /battery_level
Check low battery alerts: rostopic echo /alert_topic

Recording Bag File

To record ROS messages:
rosbag record -O turtlebot_low_battery.bag /battery_level /alert_topic

To play back the recorded bag file:
rosbag play turtlebot_low_battery.bag

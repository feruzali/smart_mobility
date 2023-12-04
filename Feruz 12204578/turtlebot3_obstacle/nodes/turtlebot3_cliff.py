"""
Author:
Feruz 12204578

Overview:
ROS node that subscribes to the TurtleBot3 sensor state topic and adjusts the linear velocity
of the robot based on cliff sensor readings to avoid falling off edges.

Node Details:
- Subscribes to 'sensor_state' topic to get cliff sensor readings.
- Publishes velocity commands on 'cmd_vel' topic to control the robot's movement.
- Adjusts linear velocity based on the cliff sensor reading:
    - If cliff distance is less than 1000 units, stops the robot (linear velocity = 0).
    - If cliff distance is greater than or equal to 1000 units, moves forward with a linear velocity of 0.05 m/s.
- Runs at a rate of 10 Hz.
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

class Cliff():
    def __init__(self):
        # Initialize ROS node, publisher, and subscriber
        rospy.init_node('turtlebot3_cliff')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cliff_sub = rospy.Subscriber('sensor_state', SensorState, self.get_cliff, queue_size=1)
        # Call the cliff method
        self.cliff()

    def get_cliff(self, sensor):
        # Update twist linear.x based on cliff sensor reading
        twist = Twist()
        if sensor.cliff < 1000:
            linear_vel = 0
        else:
            linear_vel = 0.05

        twist.linear.x = linear_vel
        self.cmd_pub.publish(twist)

    def cliff(self):
        # Main cliff method that continuously publishes Twist messages
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    # Initialize the Cliff class within the try block to handle ROSInterruptException
    try:
        cliff = Cliff()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

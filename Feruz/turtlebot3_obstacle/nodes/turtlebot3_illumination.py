"""
Author:
Feruz 12204578

Overview:
ROS node that subscribes to the TurtleBot3 sensor state topic and adjusts the linear velocity
of the robot based on the illumination sensor readings to respond to changes in light conditions.

Node Details:
- Subscribes to 'sensor_state' topic to get illumination sensor readings.
- Publishes velocity commands on 'cmd_vel' topic to control the robot's movement.
- Adjusts linear velocity based on the illumination sensor reading:
    - If illumination is less than 200 units, moves forward with a linear velocity of 0.05 m/s.
    - If illumination is greater than or equal to 200 units, stops the robot (linear velocity = 0).
- Runs at a rate of 10 Hz.
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

class Illumination():
    def __init__(self):
        # Initialize ROS node, publisher, and subscriber
        rospy.init_node('turtlebot3_illumination')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.illumination_sub = rospy.Subscriber('sensor_state', SensorState, self.get_illumination, queue_size=1)
        # Call the illumination method
        self.illumination()

    def get_illumination(self, sensor):
        # Update twist linear.x based on illumination sensor reading
        twist = Twist()
        if sensor.illumination < 200:
            linear_vel = 0.05
        else:
            linear_vel = 0

        twist.linear.x = linear_vel
        self.cmd_pub.publish(twist)

    def illumination(self):
        # Main illumination method that continuously publishes Twist messages
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    # Initialize the Illumination class within the try block to handle ROSInterruptException
    try:
        illumination = Illumination()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

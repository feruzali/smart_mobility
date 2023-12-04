"""
Author:
Feruz 12204578

Overview:
ROS node for controlling TurtleBot3's movement based on sonar sensor readings.

Node Details:
- Publishes Twist messages to 'cmd_vel' topic for controlling the robot's velocity.
- Subscribes to 'sensor_state' topic to receive sonar sensor readings.
- Adjusts the linear velocity based on the sonar reading to avoid obstacles.
"""

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

class Sonar():
    def __init__(self):
        # Initialize ROS node, publisher, and subscriber
        rospy.init_node('turtlebot3_sonar')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sonar_sub = rospy.Subscriber('sensor_state', SensorState, self.get_sonar, queue_size=1)
        # Call the sonar method to start processing sensor data
        self.sonar()

    def get_sonar(self, sensor):
        # Callback function to process sonar sensor data
        twist = Twist()
        # Adjust linear velocity based on sonar reading
        if sensor.sonar < 10:
            linear_vel = 0
        else:
            linear_vel = 0.05

        twist.linear.x = linear_vel
        # Publish the updated Twist message for velocity control
        self.cmd_pub.publish(twist)

    def sonar(self):
        # Main loop for the node
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Sleep to control the loop rate
            rate.sleep()

def main():
    try:
        # Create an instance of the Sonar class
        sonar = Sonar()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    # Call the main function when the script is executed
    main()

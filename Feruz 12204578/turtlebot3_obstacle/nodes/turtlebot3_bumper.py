"""
Author:
Feruz 12204578

Overview:
ROS node that subscribes to the TurtleBot3 sensor state topic and adjusts the linear velocity
of the robot based on bumper sensor readings.


Node Details:
- Subscribes to 'sensor_state' topic to get bumper sensor readings.
- Publishes velocity commands on 'cmd_vel' topic to control the robot's movement.
- Adjusts linear velocity based on the bumper sensor state:
    - If the left bumper is activated, moves backward with a linear velocity of -0.1 m/s.
    - If the right bumper is activated, moves forward with a linear velocity of 0.1 m/s.
- Runs at a rate of 10 Hz.
"""
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

class Bumper():
    def __init__(self):
        # Initialize ROS node, publisher, and subscriber
        rospy.init_node('turtlebot3_bumper')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.bumper_sub = rospy.Subscriber('sensor_state', SensorState, self.get_bumper, queue_size=1)
        self.twist = Twist()
        # Call the bumper method
        self.bumper()

    def get_bumper(self, sensor):
        # Update twist linear.x based on bumper state
        self.bumper_state = sensor.bumper
        if self.bumper_state == 1:
            self.twist.linear.x = -0.1
        elif self.bumper_state == 2:
            self.twist.linear.x = 0.1

    def bumper(self):
        # Main bumper method that continuously publishes Twist messages
        rate = rospy.Rate(10)
        self.twist.linear.x = 0.1
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            rate.sleep()

def main():
    # Initialize the Bumper class within the try block to handle ROSInterruptException
    try:
        bumper = Bumper()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

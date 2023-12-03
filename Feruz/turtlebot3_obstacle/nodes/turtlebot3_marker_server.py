"""
Author:
Feruz 12204578

Overview:
ROS node that creates an interactive marker server for controlling the TurtleBot3's movement and rotation
using a graphical marker in the RViz environment.

Node Details:
- Creates an interactive marker server to control the TurtleBot3 in RViz.
- Publishes Twist messages on the 'cmd_vel' topic based on the interactive marker feedback.
- Allows movement along the X-axis and rotation around the Z-axis.
"""

import rospy
from geometry_msgs.msg import Twist, Pose
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion
import copy

def processFeedback(feedback):
    # Process the interactive marker feedback and publish Twist messages for robot movement
    _, _, yaw = euler_from_quaternion((feedback.pose.orientation.x, feedback.pose.orientation.y,
                                        feedback.pose.orientation.z, feedback.pose.orientation.w))

    twist = Twist()
    twist.angular.z = 2.2 * yaw
    twist.linear.x = 1.0 * feedback.pose.position.x

    vel_pub.publish(twist)

    # Reset the marker pose to prevent accumulation of movement
    server.setPose("turtlebot3_marker", Pose())
    server.applyChanges()

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("turtlebot3_interactive_marker_server")

    # Create an interactive marker server and a Twist publisher
    server = InteractiveMarkerServer("turtlebot3_marker_server")
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    # Create an interactive marker with controls for movement along the X-axis and rotation around the Z-axis
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "turtlebot3_marker"

    # Control for moving along the X-axis
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.always_visible = True
    int_marker.controls.append(copy.deepcopy(control))

    # Control for rotating around the Z-axis
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))

    # Insert the marker and set the feedback callback
    server.insert(int_marker, processFeedback)
    server.applyChanges()

    # Spin to keep the node alive
    rospy.spin()

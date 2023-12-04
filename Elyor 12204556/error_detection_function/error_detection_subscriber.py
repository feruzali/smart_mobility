#!/usr/bin/env python

import rospy  #Importing the rospy library for ROS
from std_msgs.msg import String  #Importing the String message type

def error_callback(data):
    #Callback function triggered when an error message is received
    rospy.loginfo("Error detected: %s", data.data)  #Logging the received error message

def error_detection_subscriber():
    rospy.init_node('error_detection_subscriber')  #Initializing the ROS node with a unique name
    rospy.Subscriber('/error_detection_alert', String, error_callback)  #Subscribing to the error detection topic
    
    rospy.spin()  #Keeping the node running to listen for messages

if __name__ == '__main__':
    error_detection_subscriber()  #Running the error detection subscriber node


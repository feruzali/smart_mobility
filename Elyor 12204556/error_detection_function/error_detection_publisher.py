#!/usr/bin/env python

import rospy  #Importing the rospy library for ROS
from std_msgs.msg import String  # Importing the String message type

def error_detection_publisher():
    rospy.init_node('error_detection_publisher')  #Initializing the ROS node
    error_pub = rospy.Publisher('/error_detection_alert', String, queue_size=10)  #Creating a publisher for the alert 
    
    rate = rospy.Rate(0.2)  #Setting up the publishing rate (0.2 messages per second, i.e., every 5 seconds)
    alert_message = "Obstacle detected! Stop Turtlebot."

    while not rospy.is_shutdown():
        error_pub.publish(alert_message)  #Publish an error message
        
         #Code to stop Turtlebot movement 
        rospy.loginfo("Turtlebot stopped. Waiting for manual intervention.")
        
        rate.sleep(0.2)  #Sleeping according to the defined rate

if __name__ == '__main__':
    try:
        error_detection_publisher()  #Running the error detection publisher function
    except rospy.ROSInterruptException:
        pass


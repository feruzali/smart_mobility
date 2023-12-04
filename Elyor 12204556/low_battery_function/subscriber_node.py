#!/usr/bin/env python

#Importing necessary ROS libraries and message types
import rospy
from std_msgs.msg import Float32, String

#Defining a callback function to handle battery level messages
def battery_level_callback(data):
    #Logging battery level received from '/battery_level' topic
    rospy.loginfo("Battery Level: {}%".format(data.data))

#Defining a callback function to handle low battery alert messages
def low_battery_alert_callback(data):
    #Checking if the received message indicates a low battery alert
    if data.data == "LowBatteryAlert":
        #Logging an alert message indicating low battery
        rospy.loginfo("Audible alert triggered for low battery.")

#Defining a function for the battery level subscriber node
def battery_level_subscriber():
    #Initializing a ROS node with the name 'battery_level_subscriber'
    rospy.init_node('battery_level_subscriber')

    #Subscribing to '/battery_level' topic and define the callback function
    rospy.Subscriber('/battery_level', Float32, battery_level_callback)
    #Subscribing to '/alert_topic' topic for low battery alerts and define the callback function
    rospy.Subscriber('/alert_topic', String, low_battery_alert_callback)

    #Keeping the node running until shutdown
    rospy.spin()

if __name__ == '__main__':
    #Running the battery level subscriber function
    battery_level_subscriber()


#!/usr/bin/env python

#Importing necessary ROS libraries and message types
import rospy
from std_msgs.msg import Float32, String

#Defining a function to publish battery level and low battery alerts
def battery_level_publisher():
    #Initializing a ROS node with the name 'battery_level_publisher'
    rospy.init_node('battery_level_publisher')
    
    #Creating publishers for battery level and low battery alerts
    battery_pub = rospy.Publisher('/battery_level', Float32, queue_size=10)
    alert_pub = rospy.Publisher('/alert_topic', String, queue_size=10)

    #Setting up the publishing rate (1 message per second)
    rate = rospy.Rate(1)
    #Simulating starting battery level at 100%
    battery_level = 100.0

    #Continuously publishing battery levels until shutdown
    while not rospy.is_shutdown():
        #Publishing the current battery level
        battery_pub.publish(battery_level)

        #Checking if the battery level is below 20%
        if battery_level < 20.0:
            #Logging a message indicating low battery and initiate alert publication
            rospy.loginfo("Low battery detected! Initiating charging procedure.")
            alert_pub.publish("LowBatteryAlert")     
            #Navigating to the charging spot
            navigate_to_charging_spot()

        #Simulating decreasing battery level by 1% each iteration
        battery_level -= 1.0
        rate.sleep()  #Maintaining the publishing rate

if __name__ == '__main__':
    try:
        #Running the battery level publisher function
        battery_level_publisher()
    except rospy.ROSInterruptException:
        #Handling potential ROS interrupt exceptions 
        pass

    try:
        battery_level_publisher()
    except rospy.ROSInterruptException:
        pass


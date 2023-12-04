"""
Author:
Feruz 12204578

Overview:
ROS client script for interacting with the TurtleBot3 patrol action server.
Allows the user to input patrol mode, area, and count for the TurtleBot3 to execute.

Script Details:
- Implements a simple client to interact with the 'turtlebot3' action server.
- Accepts user input for patrol mode, area, and count.
- Sends a goal to the action server based on user input.
- Displays the result of the action server's execution.
"""

from __future__ import print_function
import rospy
import actionlib
import turtlebot3_example.msg
import sys

msg = """
patrol your TurtleBot3!
-----------------------
mode : s - Patrol to Square
       t - Patrol to Triangle
       c - Patrol to Circle

area : Square, Triangle mode - length of side (m)
       Circle mode - radius (m)

count - patrol count

If you want to close, insert 'x'
"""

class Client():
    def __init__(self):
        rospy.loginfo("Waiting for the server...")
        self.client()

    def getkey(self):
        mode, area, count = input("| mode | area | count |\n").split()
        mode, area, count = [str(mode), float(area), int(count)]

        if mode == 's':
            mode = 1
        elif mode == 't':
            mode = 2
        elif mode == 'c':
            mode = 3
        elif mode == 'x':
            self.shutdown()
        else:
            rospy.loginfo("You selected the wrong mode.")

        return mode, area, count

    def client(self):
        client = actionlib.SimpleActionClient('turtlebot3', turtlebot3_example.msg.Turtlebot3Action)

        mode, area, count = self.getkey()
        client.wait_for_server()
        goal = turtlebot3_example.msg.Turtlebot3Goal()
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count
        client.send_goal(goal)
        rospy.loginfo("Sent goal to the server.")
        client.wait_for_result()

        rospy.loginfo(client.get_result())

    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_client')
    try:
        while not rospy.is_shutdown():
            print(msg)
            result = Client()
    except:
        print("Program closed.", file=sys.stderr)

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
import click

# User prompt and information message
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
    def __init__(self, mode, area, count):
        rospy.loginfo("Waiting for the server...")
        self.client(mode, area, count)

    def client(self, mode, area, count):
        # Initialize SimpleActionClient
        client = actionlib.SimpleActionClient('turtlebot3', turtlebot3_example.msg.Turtlebot3Action)

        # Wait for the server to be available
        client.wait_for_server()

        # Create a goal and send it to the server
        goal = turtlebot3_example.msg.Turtlebot3Goal()
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count
        client.send_goal(goal)
        rospy.loginfo("Sent goal to the server.")

        # Wait for the result
        client.wait_for_result()

        # Log the result
        rospy.loginfo(client.get_result())

    def shutdown(self):
        # Gracefully shut down the node
        rospy.sleep(1)

@click.command()
@click.option('--mode', type=click.Choice(['s', 't', 'c', 'x']), prompt=True, help="Patrol mode: s, t, c, or x to close")
@click.option('--area', type=float, prompt=True, help="Area value: length of side (m) or radius (m)")
@click.option('--count', type=int, prompt=True, help="Patrol count")
def main(mode, area, count):
    # Initialize the ROS node
    rospy.init_node('turtlebot3_client')
    try:
        while not rospy.is_shutdown():
            # Display user prompt and information
            print(msg)
            # Create Client instance with provided options
            result = Client(mode, area, count)
    except:
        # Handle program closure
        print("Program closed.", file=sys.stderr)

if __name__ == '__main__':
    # Run the main function
    main()


# Author: Feruz 12204578

import rospy
import pytest
from mock import MagicMock
from turtlebot3_example.msg import Turtlebot3Action, Turtlebot3ActionGoal
from turtlebot3_action import Turtlebot3Action

# Fixture for a mock action server
@pytest.fixture
def mock_action_server():
    return MagicMock()

# Fixture for a mock rospy
@pytest.fixture
def mock_rospy():
    return MagicMock()

# Fixture for a mock action goal
@pytest.fixture
def mock_action_goal():
    return Turtlebot3ActionGoal()

# Test function for Turtlebot3Action execute_cb method
def test_turtlebot3_action_execute_cb(mock_action_server, mock_rospy, mock_action_goal):
    # Initialize a mock rospy node
    rospy.init_node('test_node')

    # Use rospy.mock_service to simulate a service for the action server
    with rospy.mock_service('turtlebot3_action_server', Turtlebot3Action):
        # Create an instance of Turtlebot3Action
        server = Turtlebot3Action('turtlebot3_action_server')

        # Call the execute_cb method with a mock action goal
        server.execute_cb(mock_action_goal)
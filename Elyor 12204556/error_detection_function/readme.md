                   Error Detection Functionality

This repository contains the error detection functionality implemented for a Turtlebot within a restaurant environment.

                    Overview

The error detection functionality aims to identify obstacles or issues encountered by the Turtlebot during its operation within the restaurant. When an obstacle is detected, the Turtlebot triggers an alert message, halts its movement, and waits for manual intervention by the restaurant staff.

                     Contents

error_detection_publisher.py: ROS node simulating the error detection alert by publishing messages on the /error_detection_alert topic.
error_detection_subscriber.py: ROS node subscribing to the /error_detection_alert topic to receive error detection messages.
error_detection.launch: Launch file to start the error detection publisher and subscriber nodes simultaneously.
test_error_detection.py: PyTest scenarios to validate the error detection start and stop functionalities

                          Usage

Installation

Clone the repository:
git clone https://github.com/feruzali/smart_mobility/tree/main/Elyor%2012204556/error_detection_function

Running the Error Detection:
roslaunch eror_detection error_detection.launch

Testing:
pytest test_error_detection.py


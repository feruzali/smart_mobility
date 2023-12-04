# Waiter Bot - Call the Waiter Function

## Overview

This repository contains the source code for the "Call the Waiter" function implemented on TurtleBot3 using ROS (Robot Operating System). The function allows customers to press a button, triggering the TurtleBot to come to their location and take their order with the help of a speech-to-text function.

## Features

- **Responsive:** TurtleBot moves to the customer's location upon button press.
- **ROS Integration:** Utilizes ROS for communication between components.

## Prerequisites

- ROS (Robot Operating System)
- TurtleBot3 packages


## Usage

1. Launch the "Call the Waiter" node:

    ```bash
    roslaunch your_package_name call_waiter.launch
    ```

2. Press the "Call the Waiter" button on the website.

3. Observe TurtleBot responding to the call and taking the order.

## Configuration

Adjust the configuration parameters in `waiter_bot.py`. Ensuring that the ROS topics for speech-to-text and button press are correctly configured.

## Referencess

- [ROS Wiki](http://wiki.ros.org/)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)


## Author

- Abror Khasanov
- Contact: abbossxon95@gmail.com



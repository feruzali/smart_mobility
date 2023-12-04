# TurtleBot3 Diagnostics Module

## Author
Feruz 12204578

## Overview
This ROS (Robot Operating System) node is designed for monitoring and diagnosing the status of TurtleBot3 robot components. The monitored components include the IMU sensor, motor actuators, Lidar sensor, power system (battery), and analog buttons. The node also compares the firmware version of the robot with the expected version and publishes diagnostic information.

## ROS Packages Used
- **sensor_msgs**: Standard ROS messages for sensor data.
- **diagnostic_msgs**: Standard ROS messages for diagnostic information.
- **turtlebot3_msgs**: Custom ROS messages specific to TurtleBot3.

## Node Functionality
1. Subscribes to sensor messages from IMU, Lidar, sensor state, and firmware version topics.
2. Monitors and diagnoses the status of various robot components.
3. Publishes diagnostic information and version information.

## Constants
- **SOFTWARE_VERSION**: Software version of this diagnostic node.
- **HARDWARE_VERSION**: Expected hardware version of the TurtleBot3 robot.
- **FIRMWARE_VERSION_MAJOR_NUMBER**: Expected major firmware version number.
- **FIRMWARE_VERSION_MINOR_NUMBER**: Expected minor firmware version number.

## Functions
- **split**: Splits a string based on a specified separator and stores the results in an array.
- **setDiagnosisMsg**: Sets diagnostic message parameters for a specific component.
- **setIMUDiagnosis, setMotorDiagnosis, setBatteryDiagnosis, setLDSDiagnosis, setButtonDiagnosis**: Convenience functions to set diagnostic messages for specific components.
- **imuMsgCallback, LDSMsgCallback, sensorStateMsgCallback, firmwareVersionMsgCallback**: Callback functions to handle incoming messages from subscribed topics.
- **msgPub**: Publishes the diagnostic array containing information about different robot components.

## Main Function
- Initializes ROS, sets up publishers and subscribers.
- Enters a loop to continuously publish diagnostic information and handle callbacks.

## ROS Node Execution
```bash
rosrun turtlebot3_diagnostics turtlebot3_diagnostics
```

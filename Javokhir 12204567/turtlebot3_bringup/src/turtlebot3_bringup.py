Author: Javokhir Jambulov

#!/usr/bin/env python

import rospy
import click
import pytest
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from turtlebot3_msgs.msg import SensorState, VersionInfo

SOFTWARE_VERSION = "1.2.5"
HARDWARE_VERSION = "2020.03.16"
FIRMWARE_VERSION_MAJOR_NUMBER = 1
FIRMWARE_VERSION_MINOR_NUMBER = 2

tb3_version_info_pub = rospy.Publisher('version_info', VersionInfo, queue_size=10)
tb3_diagnostics_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)

imu_state = DiagnosticStatus()
motor_state = DiagnosticStatus()
LDS_state = DiagnosticStatus()
battery_state = DiagnosticStatus()
button_state = DiagnosticStatus()

class VERSION:
    pass

def split(data, separator):
    temp = []
    cnt = 0
    copy = data

    while True:
        index = copy.find(separator)

        if index != -1:
            temp.append(copy[:index])
            copy = copy[index+1:]
        else:
            temp.append(copy)
            break

        cnt += 1

    return temp

def set_diagnosis_msg(diag, level, name, message, hardware_id):
    diag.level = level
    diag.name = name
    diag.message = message
    diag.hardware_id = hardware_id

def set_imu_diagnosis(level, message):
    set_diagnosis_msg(imu_state, DiagnosticStatus.OK, "IMU Sensor", message, "MPU9250")

def set_motor_diagnosis(level, message):
    set_diagnosis_msg(motor_state, DiagnosticStatus.OK, "Actuator", message, "DYNAMIXEL X")

def set_battery_diagnosis(level, message):
    set_diagnosis_msg(battery_state, DiagnosticStatus.OK, "Power System", message, "Battery")

def set_LDS_diagnosis(level, message):
    set_diagnosis_msg(LDS_state, DiagnosticStatus.OK, "Lidar Sensor", message, "HLS-LFCD-LDS")

def set_button_diagnosis(level, message):
    set_diagnosis_msg(button_state, DiagnosticStatus.OK, "Analog Button", message, "OpenCR Button")

def imu_msg_callback(msg):
    set_imu_diagnosis(DiagnosticStatus.OK, "Good Condition")

def LDS_msg_callback(msg):
    set_LDS_diagnosis(DiagnosticStatus.OK, "Good Condition")

def sensor_state_msg_callback(msg):
    global battery_state, button_state, motor_state

    if msg.battery > 11.0:
        set_battery_diagnosis(DiagnosticStatus.OK, "Good Condition")
    else:
        set_battery_diagnosis(DiagnosticStatus.WARN, "Charge!!! Charge!!!")

    if msg.button == SensorState.BUTTON0:
        set_button_diagnosis(DiagnosticStatus.OK, "BUTTON 0 IS PUSHED")
    elif msg.button == SensorState.BUTTON1:
        set_button_diagnosis(DiagnosticStatus.OK, "BUTTON 1 IS PUSHED")
    else:
        set_button_diagnosis(DiagnosticStatus.OK, "Pushed Nothing")

    if msg.torque:
        set_motor_diagnosis(DiagnosticStatus.OK, "Torque ON")
    else:
        set_motor_diagnosis(DiagnosticStatus.WARN, "Torque OFF")

def firmware_version_msg_callback(msg):
    global check_version

    if not hasattr(firmware_version_msg_callback, 'check_version'):
        check_version = False

    get_version = split(msg.firmware, ".")
    firmware_version = VERSION()
    firmware_version.major_number = int(get_version[0])
    firmware_version.minor_number = int(get_version[1])
    firmware_version.patch_number = int(get_version[2])

    if not check_version:
        if firmware_version.major_number == FIRMWARE_VERSION_MAJOR_NUMBER:
            if firmware_version.minor_number > FIRMWARE_VERSION_MINOR_NUMBER:
                rospy.logwarn("This firmware(v%s) may not be compatible with this software (v%s)", msg.firmware, SOFTWARE_VERSION)
                rospy.logwarn("You can find how to update it in the `FAQ` section(turtlebot3.robotis.com)")
        else:
            rospy.logwarn("This firmware(v%s) may not be compatible with this software (v%s)", msg.firmware, SOFTWARE_VERSION)
            rospy.logwarn("You can find how to update it in the `FAQ` section(turtlebot3.robotis.com)")

        check_version = True

    version = VersionInfo()
    version.software = SOFTWARE_VERSION
    version.hardware = HARDWARE_VERSION
    version.firmware = msg.firmware

    tb3_version_info_pub.publish(version)

def msg_pub():
    global imu_state, motor_state, LDS_state, battery_state, button_state

    tb3_diagnostics = DiagnosticArray()
    tb3_diagnostics.header.stamp = rospy.Time.now()

    tb3_diagnostics.status.clear()
    tb3_diagnostics.status.extend([imu_state, motor_state, LDS_state, battery_state, button_state])

    tb3_diagnostics_pub.publish(tb3_diagnostics)

@click.command()
@click.option('--test', is_flag=True, help='Run pytest test')
def main(test):
    global tb3_diagnostics_pub, tb3_version_info_pub

    if test:
        pytest.main(['-v', __file__])
    else:
        rospy.init_node('turtlebot3_diagnostic')
        nh = rospy.NodeHandle()

        tb3_diagnostics_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)
        tb3_version_info_pub = rospy.Publisher('version_info', VersionInfo, queue_size=10)

        rospy.Subscriber('imu', Imu, imu_msg_callback)
        rospy.Subscriber('scan', LaserScan, LDS_msg_callback)
        rospy.Subscriber('sensor_state', SensorState, sensor_state_msg_callback)
        rospy.Subscriber('firmware_version', VersionInfo, firmware_version_msg_callback)

        loop_rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            msg_pub()
            loop_rate.sleep()

if __name__ == '__main__':
    main()

/* 
  Author: Feruz 12204578
  
  Overview: 
  This ROS (Robot Operating System) node monitors and diagnoses the status of TurtleBot3 robot components, 
  including IMU sensor, motor actuators, Lidar sensor, power system (battery), and analog buttons. 
  It also compares the firmware version of the robot with the expected version and publishes diagnostic information.

  ROS Packages Used:
  - sensor_msgs: Standard ROS messages for sensor data.
  - diagnostic_msgs: Standard ROS messages for diagnostic information.
  - turtlebot3_msgs: Custom ROS messages specific to TurtleBot3.

  Node Functionality:
  1. Subscribes to sensor messages from IMU, Lidar, sensor state, and firmware version topics.
  2. Monitors and diagnoses the status of various robot components.
  3. Publishes diagnostic information and version information.

  Constants:
  - SOFTWARE_VERSION: Software version of this diagnostic node.
  - HARDWARE_VERSION: Expected hardware version of the TurtleBot3 robot.
  - FIRMWARE_VERSION_MAJOR_NUMBER: Expected major firmware version number.
  - FIRMWARE_VERSION_MINOR_NUMBER: Expected minor firmware version number.

  Custom Struct:
  - VERSION: Structure to store major, minor, and patch numbers of firmware version.

  Functions:
  - split: Splits a string based on a specified separator and stores the results in an array.
  - setDiagnosisMsg: Sets diagnostic message parameters for a specific component.
  - setIMUDiagnosis, setMotorDiagnosis, setBatteryDiagnosis, setLDSDiagnosis, setButtonDiagnosis: 
    Convenience functions to set diagnostic messages for specific components.
  - imuMsgCallback, LDSMsgCallback, sensorStateMsgCallback, firmwareVersionMsgCallback: 
    Callback functions to handle incoming messages from subscribed topics.
  - msgPub: Publishes the diagnostic array containing information about different robot components.
  
  Main Function:
  - Initializes ROS, sets up publishers and subscribers.
  - Enters a loop to continuously publish diagnostic information and handle callbacks.
*/

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <string>

#define SOFTWARE_VERSION "1.0.0"
#define HARDWARE_VERSION "2023.09.16"
#define FIRMWARE_VERSION_MAJOR_NUMBER 1
#define FIRMWARE_VERSION_MINOR_NUMBER 2

ros::Publisher tb3_version_info_pub;
ros::Publisher tb3_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus LDS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;

typedef struct
{
  int major_number;
  int minor_number;
  int patch_number;
} VERSION;

// Function to split a string into an array based on a separator
void split(std::string data, std::string separator, std::string* temp)
{
  int cnt = 0;
  std::string copy = data;

  while (true)
  {
    std::size_t index = copy.find(separator);

    if (index != std::string::npos)
    {
      temp[cnt] = copy.substr(0, index);
      copy = copy.substr(index + 1, copy.length());
    }
    else
    {
      temp[cnt] = copy.substr(0, copy.length());
      break;
    }

    ++cnt;
  }
}

// Function to set diagnostic message parameters
void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus* diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

// Convenience functions to set diagnostic messages for specific components
void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "DYNAMIXEL X");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS");
}

void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "OpenCR Button");
}

// Callback functions to handle incoming messages from subscribed topics
void imuMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const turtlebot3_msgs::SensorState::ConstPtr& msg)
{
  if (msg->battery > 11.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  if (msg->button == turtlebot3_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == turtlebot3_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  if (msg->torque == true)
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Torque OFF");
}

void firmwareVersionMsgCallback(const turtlebot3_msgs::VersionInfo::ConstPtr& msg)
{
  static bool check_version = false;
  std::string get_version[3];

  split(msg->firmware, ".", get_version);

  VERSION firmware_version;
  firmware_version.major_number = std::stoi(get_version[0]);
  firmware_version.minor_number = std::stoi(get_version[1]);
  firmware_version.patch_number = std::stoi(get_version[2]);

  if (check_version == false)
  {
    if (firmware_version.major_number == FIRMWARE_VERSION_MAJOR_NUMBER)
    {
      if (firmware_version.minor_number > FIRMWARE_VERSION_MINOR_NUMBER)
      {
        ROS_WARN("This firmware(v%s) may not be compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
        ROS_WARN("You can find how to update it in the `FAQ` section (turtlebot3.robotis.com)");
      }
    }
    else
    {
      ROS_WARN("This firmware(v%s) may not be compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
      ROS_WARN("You can find how to update it in the `FAQ` section (turtlebot3.robotis.com)");
    }

    check_version = true;
  }

  turtlebot3_msgs::VersionInfo version;

  version.software = SOFTWARE_VERSION;
  version.hardware = HARDWARE_VERSION;
  version.firmware = msg->firmware;

  tb3_version_info_pub.publish(version);
}

// Function to publish diagnostic information
void msgPub()
{
  diagnostic_msgs::DiagnosticArray tb3_diagnostics;

  tb3_diagnostics.header.stamp = ros::Time::now();

  tb3_diagnostics.status.clear();
  tb3_diagnostics.status.push_back(imu_state);
  tb3_diagnostics.status.push_back(motor_state);
  tb3_diagnostics.status.push_back(LDS_state);
  tb3_diagnostics.status.push_back(battery_state);
  tb3_diagnostics.status.push_back(button_state);

  tb3_diagnostics_pub.publish(tb3_diagnostics);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot3_diagnostic");
  ros::NodeHandle nh;

  tb3_diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  tb3_version_info_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("version_info", 10);

  ros::Subscriber imu = nh.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds = nh.subscribe("scan", 10, LDSMsgCallback);
  ros::Subscriber tb3_sensor = nh.subscribe("sensor_state", 10, sensorStateMsgCallback);
  ros::Subscriber version = nh.subscribe("firmware_version", 10, firmwareVersionMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

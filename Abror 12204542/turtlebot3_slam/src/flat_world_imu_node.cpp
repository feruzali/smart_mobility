/*
Author:
Abror 12204542

Overview:
ROS node that modifies IMU messages to simulate a flat world, setting linear acceleration
in the z-axis to GRAVITY. It subscribes to the 'imu_in' topic, processes incoming IMU messages,
and publishes the modified messages on the 'imu_out' topic.

Node Details:
- Advertises 'imu_out' topic to publish modified IMU messages.
- Subscribes to 'imu_in' topic to receive incoming IMU messages.
- Modifies linear acceleration in the z-axis to simulate a flat world.
- Runs the ROS event loop.

Parameters:
- GRAVITY: Constant representing gravitational acceleration in the z-axis.

Usage:
- Run the node using 'rosrun <package_name> <executable_name>'.
*/

#include <turtlebot3_slam/flat_world_imu_node.h>

// Constructor
FlatWorldImuNode::FlatWorldImuNode()
{
  // Initialize the node and check for successful initialization
  bool init_result = init();
  ROS_ASSERT(init_result);
}

// Destructor
FlatWorldImuNode::~FlatWorldImuNode()
{
  // Destructor - nothing specific to clean up
}

// Initialization function
bool FlatWorldImuNode::init()
{
  // Advertise the 'imu_out' topic and subscribe to the 'imu_in' topic
  publisher_  = nh_.advertise<sensor_msgs::Imu>("imu_out", 10);
  subscriber_ = nh_.subscribe("imu_in", 150, &FlatWorldImuNode::msgCallback, this);

  // Initialization successful
  return true;
}

// Callback function for the 'imu_in' topic
void FlatWorldImuNode::msgCallback(const sensor_msgs::ImuConstPtr imu_in)
{
  // Process the incoming IMU message

  // Check if the current message has a later timestamp than the last published message
  if (last_published_time_.isZero() || imu_in->header.stamp > last_published_time_)
  {
    // Update the last published time
    last_published_time_ = imu_in->header.stamp;

    // Create a copy of the incoming IMU message
    sensor_msgs::Imu imu_out = *imu_in;

    // Set linear acceleration in the flat world (z-axis) to GRAVITY
    imu_out.linear_acceleration.x = 0.0;
    imu_out.linear_acceleration.y = 0.0;
    imu_out.linear_acceleration.z = GRAVITY;

    // Publish the modified IMU message
    publisher_.publish(imu_out);
  }
}

// Main function
int main(int argc, char* argv[])
{
  // Initialize the ROS node
  ros::init(argc, argv, "flat_world_imu_node");

  // Create an instance of the FlatWorldImuNode class
  FlatWorldImuNode flat_world_imu_node;

  // Start the ROS event loop
  ros::spin();

  // Return success
  return 0;
}

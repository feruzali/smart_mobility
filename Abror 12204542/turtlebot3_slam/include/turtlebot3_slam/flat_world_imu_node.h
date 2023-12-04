// Abror 12204542

#ifndef FLAT_WORLD_IMU_NODE_H_
#define FLAT_WORLD_IMU_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Define a constant for gravitational acceleration
#define GRAVITY 9.8

/**
 * @brief ROS node for transforming IMU data in a flat world assumption.
 *
 * The FlatWorldImuNode class subscribes to the 'imu_in' topic, assumes a flat world,
 * and publishes the transformed IMU data on the 'imu_out' topic. The linear acceleration
 * along the z-axis is set to a constant value representing Earth's gravity.
 *
 * Usage:
 * - Instantiate the FlatWorldImuNode class.
 * - Initialize the node.
 * - The node listens to 'imu_in', transforms the data, and publishes it on 'imu_out'.
 */
class FlatWorldImuNode
{
 public:
  // Constructor
  FlatWorldImuNode();

  // Destructor
  ~FlatWorldImuNode();

  // Initialization method
  bool init();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // Last published timestamp
  ros::Time last_published_time_;

  // ROS Publisher for 'imu_out' topic
  ros::Publisher publisher_;

  // ROS Subscriber for 'imu_in' topic
  ros::Subscriber subscriber_;

  // Callback function for 'imu_in' topic
  void msgCallback(const sensor_msgs::ImuConstPtr imu_in);
};

#endif // FLAT_WORLD_IMU_NODE_H_

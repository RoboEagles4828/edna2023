#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/**
 * This tutorial demonstrates simple receipt of IMU sensor data over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "imu_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);

  ros::spin();

  return 0;
}

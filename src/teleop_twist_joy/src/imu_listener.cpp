#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * This tutorial demonstrates simple receipt of IMU sensor data over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const nav_msgs::msg::Odometry msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}

int main(int argc, char **argv)
{

  const nav_msgs::msg::Odometry empty_odometry;
  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  odom_subscriber = node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_, qos,
    [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) -> void
    {
      if (!subscriber_is_active_) {
        RCLCPP_WARN( rclcpp::get_logger("isaac_hardware_interface"), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_joint_msg_ptr_.set(std::move(msg));
    });
  

  return 0;
}

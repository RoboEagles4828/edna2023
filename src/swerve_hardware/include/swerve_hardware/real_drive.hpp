// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SWERVE_HARDWARE__REAL_DRIVE_HPP_
#define SWERVE_HARDWARE__REAL_DRIVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "swerve_hardware/visibility_control.h"

namespace swerve_hardware
{
class RealDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealDriveHardware)

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double RIO_CONVERSION_FACTOR = 10000.0;
  // Store the command for the simulated robot
  std::vector<double> hw_command_velocity_;
  std::vector<double> hw_command_position_;

  // Output Topic Vectors
  std::vector<std::string> arm_names_output_;
  std::vector<double> hw_command_arm_velocity_output_;
  std::vector<double> hw_command_arm_position_output_;
  std::vector<std::string> drive_names_output_;
  std::vector<double> hw_command_drive_velocity_output_;
  std::vector<double> hw_command_drive_position_output_;

  // The state vectors
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_positions_input_;
  std::vector<double> hw_velocities_input_;

  // Mimic Joints for joints not controlled by the real robot
  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
    double offset = 0.0;
  };
  std::vector<MimicJoint> mimic_joints_;
  double parse_double(const std::string & text);
  bool parse_bool(const std::string & text);

  // Keep Track of Arm vs Drive Joints
  struct JointGroupMember
  {
    std::size_t joint_index;
    std::string joint_name;
    bool percent = false;
    double min = -1.0;
    double max = 1.0;
  };
  std::vector<JointGroupMember> drive_joints_;
  std::vector<JointGroupMember> arm_joints_;

  // Joint name array will align with state and command interface array
  // The command at index 3 of hw_command_ will be the joint name at index 3 of joint_names
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_types_;

  // Pub Sub to real
  std::string joint_state_topic_ = "real_joint_states";
  std::string joint_command_topic_ = "real_joint_commands";
  std::string joint_arm_command_topic_ = "real_arm_commands";
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> real_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_real_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> real_arm_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_real_arm_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>> received_joint_msg_ptr_{nullptr};

  // Converts isaac position range -2pi - 2pi into expected ros position range -pi - pi
  double convertToRosPosition(double real_position);
  double convertToRosVelocity(double real_velocity);
  // void convertToRealPositions(std::vector<double> ros_positions);
};

}  // namespace swerve_hardware

#endif  // SWERVE_HARDWARE__DIFFBOT_SYSTEM_HPP_
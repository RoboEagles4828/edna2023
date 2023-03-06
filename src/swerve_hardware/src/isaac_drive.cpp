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

#include "swerve_hardware/isaac_drive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swerve_hardware/motion_magic.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;

namespace swerve_hardware
{

hardware_interface::CallbackReturn IsaacDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  node_ = rclcpp::Node::make_shared("isaac_hardware_interface");

  // PUBLISHER SETUP
  isaac_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_command_topic_, rclcpp::SystemDefaultsQoS());
  realtime_isaac_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      isaac_publisher_);


  // SUBSCRIBER SETUP
  const sensor_msgs::msg::JointState empty_joint_state;
  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  received_joint_msg_ptr_.set(std::make_shared<sensor_msgs::msg::JointState>(empty_joint_state));
  isaac_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_, qos,
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
    {
      if (!subscriber_is_active_) {
        RCLCPP_WARN( rclcpp::get_logger("isaac_hardware_interface"), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_joint_msg_ptr_.set(std::move(msg));
    });
  
  // COMMON INTERFACE SETUP
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 8 positions states, 4 axle positions 4 wheel positions
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // 8 velocity states, 4 axle velocity 4 wheel velocity
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // 4 wheel velocity commands
  // We will keep this at 8 and make the other 4 zero to keep indexing consistent
  hw_command_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // 4 axle position commands
  // We will keep this at 8 and make the other 4 zero to keep indexing consistent
  hw_command_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_types_.resize(info_.joints.size(), "");
  motion_magic_.resize(info_.joints.size(), MotionMagic(MAX_ACCELERATION, MAX_VELOCITY));

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> IsaacDriveHardware::export_state_interfaces()
{
  // Each joint has 2 state interfaces: position and velocity
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> IsaacDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    auto joint = info_.joints[i];
    RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Joint Name %s", joint.name.c_str());

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str() );
      joint_types_[i] = hardware_interface::HW_IF_VELOCITY;

      // Add the command interface with a pointer to i of vel commands
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));

      // Make i of the pos command interface 0.0
      hw_command_position_[i] = 0.0;

    } else {
      RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Added Position Joint: %s", joint.name.c_str() );
      joint_types_[i] = hardware_interface::HW_IF_POSITION;

      // Add the command interface with a pointer to i of pos commands
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_command_position_[i]));

      // Make i of the pos command interface 0.0
      hw_command_velocity_[i] = 0.0;
    }
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn IsaacDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Activating ...please wait...");
  // Set Default Values for State Interface Arrays
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_command_velocity_[i] = 0.0;
    hw_command_position_[i] = 0.0;
  }
  subscriber_is_active_ = true;
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn IsaacDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Deactivating ...please wait...");
  subscriber_is_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/

double IsaacDriveHardware::convertToRosPosition(double isaac_position)
{
  // Isaac goes from -2pi to 2pi, we want -pi to pi
  if (isaac_position > M_PI) {
    return isaac_position - 2.0 * M_PI;
  } else if (isaac_position < -M_PI) {
    return isaac_position + 2.0 * M_PI;
  }
  return isaac_position;
}

hardware_interface::return_type IsaacDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);
  std::shared_ptr<sensor_msgs::msg::JointState> last_command_msg;
  received_joint_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(rclcpp::get_logger("IsaacDriveHardware"), "[%f] Velocity message received was a nullptr.", time.seconds());
    return hardware_interface::return_type::ERROR;
  }

  auto names = last_command_msg->name;
  auto positions = last_command_msg->position;
  auto velocities = last_command_msg->velocity;

  for (auto i = 0u; i < joint_names_.size(); i++) {
    for (auto j = 0u; j < names.size(); j++) {
      if (strcmp(names[j].c_str(), info_.joints[i].name.c_str()) == 0) {
        hw_positions_[i] = convertToRosPosition(positions[j]);
        hw_velocities_[i] = (float)velocities[j];
        break;
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type swerve_hardware::IsaacDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  // Calculate Axle Velocities using motion magic
  double dt = 0.1;
  for (auto i = 0u; i < joint_names_.size(); i++) {
    if (joint_names_[i].find("axle") != std::string::npos) {
      auto vel = motion_magic_[i].getNextVelocity(hw_command_position_[i], hw_positions_[i], hw_velocities_[i], dt);
      // RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Current: %f, Target: %f Vel: %f", hw_positions_[i], hw_command_position_[i], vel);
      hw_command_velocity_[i] = vel;
      hw_positions_[i] = hw_positions_[i] + hw_velocities_[i] * dt;
      auto& clk = *node_->get_clock();
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("IsaacDriveHardware"), clk, 500,
      "Joint: %s Current: %f Target: %f Vel: %f", joint_names_[i].c_str(), hw_positions_[i], hw_command_position_[i], hw_command_velocity_[i]);
    }
  }
  

  // Publish to Isaac
  if (realtime_isaac_publisher_->trylock()) {
    auto & realtime_isaac_command_ = realtime_isaac_publisher_->msg_;
    realtime_isaac_command_.header.stamp = node_->get_clock()->now();
    realtime_isaac_command_.name = joint_names_;
    realtime_isaac_command_.velocity = hw_command_velocity_;
    realtime_isaac_command_.position = empty_;
    realtime_isaac_publisher_->unlockAndPublish();
  }
  rclcpp::spin_some(node_);
  if (realtime_isaac_publisher_->trylock()) {
    auto & realtime_isaac_command_ = realtime_isaac_publisher_->msg_;
    realtime_isaac_command_.header.stamp = node_->get_clock()->now();
    realtime_isaac_command_.name = joint_names_;
    realtime_isaac_command_.velocity = empty_;
    realtime_isaac_command_.position = hw_command_position_;
    realtime_isaac_publisher_->unlockAndPublish();
  }
  rclcpp::spin_some(node_);

  return hardware_interface::return_type::OK;
}

}  // namespace swerve_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  swerve_hardware::IsaacDriveHardware, hardware_interface::SystemInterface)
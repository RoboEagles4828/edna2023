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

#include "swerve_hardware/real_drive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;
namespace swerve_hardware
{
  hardware_interface::CallbackReturn RealDriveHardware::on_init(const hardware_interface::HardwareInfo &info)
  {

    // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    // custom_qos_profile.depth = 7;

    node_ = rclcpp::Node::make_shared("real_hardware_interface");

    // PUBLISHER SETUP
    real_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("real_joint_commands", rclcpp::SystemDefaultsQoS());
    realtime_real_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        real_publisher_);

    // SUBSCRIBER SETUP
    const sensor_msgs::msg::JointState empty_joint_state;
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    received_joint_msg_ptr_.set(std::make_shared<sensor_msgs::msg::JointState>(empty_joint_state));
    real_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("real_joint_states", qos,
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(rclcpp::get_logger("real_hardware_interface"), "Can't accept new commands. subscriber is inactive");
          return;
        }
        received_joint_msg_ptr_.set(std::move(msg));
      });

    // INTERFACE SETUP
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      joint_names_.push_back(joint.name);
      // if (joint.command_interfaces.size() != 1 && (joint.name.find("wheel")))
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("RealDriveHardware"),
      //       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      //       joint.command_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
      // if (joint.command_interfaces.size() != 2 && (joint.name.find("axle")))
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("RealDriveHardware"),
      //       "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
      //       joint.command_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RealDriveHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RealDriveHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RealDriveHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RealDriveHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RealDriveHardware::export_state_interfaces()
  {
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

  std::vector<hardware_interface::CommandInterface> RealDriveHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Joint Name %s", info_.joints[i].name.c_str());

      for (auto j = 0u; j < info_.joints[i].command_interfaces.size(); j++)
      {
        if (info_.joints[i].command_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
        {
          command_interfaces.emplace_back(hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));
          RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Velocity: %s", info_.joints[i].name.c_str());
        }
        else
        {
          command_interfaces.emplace_back(hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_position_[i]));
          RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Position: %s", info_.joints[i].name.c_str());
        }
      }
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RealDriveHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Activating ...please wait...");

    // Set Default Values for State Interface Arrays
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
    }

    // Set Default Values for Command Interface Arrays
    for (auto i = 0u; i < hw_command_velocity_.size(); i++)
    {
      hw_command_velocity_[i] = 0;
    }
    for (auto i = 0u; i < hw_command_position_.size(); i++)
    {
      hw_command_position_[i] = 0;
    }

    subscriber_is_active_ = true;

    RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RealDriveHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Deactivating ...please wait...");
    subscriber_is_active_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ||                        ||
  // \/ THE STUFF THAT MATTERS \/

  hardware_interface::return_type RealDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    rclcpp::spin_some(node_);
    std::shared_ptr<sensor_msgs::msg::JointState> last_command_msg;
    received_joint_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(rclcpp::get_logger("RealDriveHardware"), "Velocity message received was a nullptr.");
      return hardware_interface::return_type::ERROR;
    }

    auto names = last_command_msg->name;
    auto positions = last_command_msg->position;
    auto velocities = last_command_msg->velocity;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      for (auto j = 0u; j < names.size(); j++)
      {
        if (strcmp(names[j].c_str(), info_.joints[i].name.c_str()) == 0)
        {
          auto radians = (float)positions[j] / 10000.0;
          // Incoming range is 0 - 2pi, convert to -pi to pi
          if (radians > M_PI){
            radians -= 2.0 * M_PI;
          }  
          hw_positions_[i] = radians;
          hw_velocities_[i] = (float)velocities[j] / 10000.0;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type swerve_hardware::RealDriveHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // Publish Velocity and Position
    if (realtime_real_publisher_->trylock())
    {
      auto &realtime_real_command_ = realtime_real_publisher_->msg_;
      realtime_real_command_.header.stamp = node_->get_clock()->now();
      realtime_real_command_.name = joint_names_;
      realtime_real_command_.velocity = hw_command_velocity_;
      realtime_real_command_.position = hw_command_position_;
      realtime_real_publisher_->unlockAndPublish();
    }
    rclcpp::spin_some(node_);

    return hardware_interface::return_type::OK;
  }

} // namespace swerve_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    swerve_hardware::RealDriveHardware, hardware_interface::SystemInterface)
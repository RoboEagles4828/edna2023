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

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swerve_hardware/motion_magic.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;

namespace swerve_hardware
{

  hardware_interface::CallbackReturn RealDriveHardware::on_init(const hardware_interface::HardwareInfo &info)
  {

    node_ = rclcpp::Node::make_shared("RealDriveHardware");

    // PUBLISHER SETUP
    real_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_command_topic_, rclcpp::SystemDefaultsQoS());
    realtime_real_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        real_publisher_);

    // SUBSCRIBER SETUP
    const sensor_msgs::msg::JointState empty_joint_state;
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    received_joint_msg_ptr_.set(std::make_shared<sensor_msgs::msg::JointState>(empty_joint_state));
    real_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_, qos, [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(rclcpp::get_logger("RealDriveHardware"), "Can't accept new commands. subscriber is inactive");
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
    hw_command_position_converted_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
    joint_types_.resize(info_.joints.size(), "");

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      joint_names_.push_back(joint.name);
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RealDriveHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

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

  std::vector<hardware_interface::CommandInterface> RealDriveHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      auto joint = info_.joints[i];
      RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Joint Name %s", joint.name.c_str());

      if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_names_output_.push_back(joint.name);
        hw_command_position_output_.push_back(0.0);
        hw_command_velocity_output_.push_back(0.0);

        RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str());
        joint_types_[i] = hardware_interface::HW_IF_VELOCITY;

        // Add the command interface with a pointer to i of vel commands
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));

        // Make i of the pos command interface 0.0
        hw_command_position_[i] = 0.0;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Added Position  Joint: %s", joint.name.c_str());
        joint_types_[i] = hardware_interface::HW_IF_POSITION;

        // Add the command interface with a pointer to i of pos commands
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_POSITION, &hw_command_position_[i]));

        // Make i of the pos command interface 0.0
        hw_command_velocity_[i] = 0.0;
        if (joint.name.find("elevator_left_elevator_center_joint") != std::string::npos || joint.name.find("arm_roller_bar_joint") != std::string::npos || joint.name.find("axle") != std::string::npos)
        {
          joint_names_output_.push_back(joint.name);
          RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Added Position Simplified Joint: %s", joint.name.c_str());
          // Add the command interface with a pointer to i of pos commands
          hw_command_position_output_.push_back(0.0);
          hw_command_velocity_output_.push_back(0.0);
        }
      }
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RealDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Activating ...please wait...");
    // Set Default Values for State Interface Arrays
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      hw_command_velocity_[i] = 0.0;
      hw_command_position_[i] = 0.0;
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

  double RealDriveHardware::convertToRosPosition(double real_position)
  {
    // Convert the rio integer that has been scaled
    real_position /= RIO_CONVERSION_FACTOR;
    // Just in case we get values we are not expecting
    real_position = std::fmod(real_position, 2.0 * M_PI);

    // Real goes from -2pi to 2pi, we want -pi to pi
    if (real_position > M_PI)
    {
      real_position -= 2.0 * M_PI;
    }
    return real_position;
  }

  double RealDriveHardware::convertToRosVelocity(double real_velocity)
  {
    // Convert the rio integer that has been scaled
    return real_velocity / RIO_CONVERSION_FACTOR;
  }
  void RealDriveHardware::convertToRealElevatorPosition()
  // std::vector<std::string> joint_names,std::vector<std::string> simplified_joint_names, std::vector<double, std::allocator<double>> hw_commmand_position, std::vector<double, std::allocator<double>> hw_command_position_output_
  {
    double add_command = 0.0;
    double rotation_pos = 0.0;
    std::vector<double> hw_command_velocity_values;
    int hw_command_value_counter = 0;
    for (auto i = 0u; i < joint_names_.size(); i++)
    {

      if (joint_names_[i].find("arm_roller_bar_joint") != std::string::npos)
      {
        rotation_pos = hw_command_position_converted_[i];
      }
      else if (joint_names_[i].find("elevator_left_elevator_center_joint") != std::string::npos)
      {
        add_command += hw_command_position_converted_[i];
      }
      else if (joint_names_[i].find("elevator_left_elevator_outer_2_joint") != std::string::npos)
      {
        add_command += hw_command_position_converted_[i];
      }

      else if (joint_names_[i].find("elevator") == std::string::npos)
      {
        hw_command_velocity_values.push_back(hw_command_velocity_[i]);
      }
    }
    // hw_commmand_position[elevator_command_iterator_position] = add_command/2.0;
    for (auto i = 0u; i < joint_names_output_.size(); i++)
    {
      if (joint_names_output_[i].find("arm_roller_bar_joint") != std::string::npos)
      {
        hw_command_position_output_[i] = rotation_pos;
      }
      else if (joint_names_output_[i].find("elevator_left_elevator_center_joint") != std::string::npos)
      {
        hw_command_position_output_[i] = add_command;
      }
      else
      {
        hw_command_velocity_output_[i] = hw_command_velocity_values[hw_command_value_counter];
        hw_command_value_counter++;
      }
    }
  }
  std::vector<std::string> RealDriveHardware::convertToRosElevatorPosition( std::vector<std::string> joint_names_input)
  {
   
    for (auto i = 0u; i < joint_names_input.size(); i++)
    {
      double ros_input_position = convertToRosPosition(hw_positions_input_[i]);
      double ros_input_velocity = convertToRosVelocity(hw_velocities_input_[i]);

      if (joint_names_input[i].find("arm_roller_bar_joint") != std::string::npos)
      {
        
        joint_names_input.push_back("elevator_left_elevator_outer_1_joint");
        hw_positions_input_.push_back(ros_input_position*1.8);
        joint_names_input.push_back("elevator_right_elevator_outer_1_joint");
        hw_positions_input_.push_back(ros_input_position*1.8);
        hw_velocities_input_.push_back(ros_input_velocity);
        hw_velocities_input_.push_back(ros_input_velocity);
      }
      else if (joint_names_input[i].find("elevator_left_elevator_center_joint") != std::string::npos)
      {
        if(hw_velocities_input_[i]>1.0)
        {
          joint_names_input.push_back("elevator_left_elevator_outer_2_joint");
          hw_positions_input_.push_back(1.0);
          hw_velocities_input_.push_back(ros_input_velocity);
          joint_names_input.push_back("elevator_right_elevator_outer_2_joint");
          hw_positions_input_.push_back(1.0);
          hw_velocities_input_.push_back(ros_input_velocity);
          joint_names_input.push_back("elevator_right_elevator_center_joint");
          hw_positions_input_.push_back(ros_input_position-1.0);
          hw_velocities_input_.push_back(ros_input_velocity);
          hw_positions_input_[i] = (ros_input_position-1.0);
          hw_velocities_input_[i] = (0.0);


        }
        else{
          joint_names_input.push_back("elevator_left_elevator_outer_2_joint");
          hw_positions_input_.push_back(ros_input_position);
          hw_velocities_input_.push_back(1.0);
          joint_names_input.push_back("elevator_right_elevator_outer_2_joint");
          hw_positions_input_.push_back(ros_input_position);
          hw_velocities_input_.push_back(1.0);
          joint_names_input.push_back("elevator_right_elevator_center_joint");
          hw_positions_input_.push_back(0.0);
          hw_velocities_input_.push_back(0.0);
          hw_positions_input_[i] = (0.0);
          hw_velocities_input_[i] = (0.0);

        }
      }
    }
    return joint_names_input;

  }


  void RealDriveHardware::convertToRealPositions(std::vector<double> ros_positions)
  {
    for (auto i = 0u; i < ros_positions.size(); i++)
    {
      // Try not to modify the original vector
      auto pos = ros_positions[i];
      if (pos < 0.0)
      {
        pos += 2.0 * M_PI;
      }
      hw_command_position_converted_[i] = pos;
    }
  }

  hardware_interface::return_type RealDriveHardware::read(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    rclcpp::spin_some(node_);
    std::shared_ptr<sensor_msgs::msg::JointState> last_command_msg;
    received_joint_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(rclcpp::get_logger("RealDriveHardware"), "[%f] Velocity message received was a nullptr.", time.seconds());
      return hardware_interface::return_type::ERROR;
    }

    std::vector<std::string> joint_names_input_ = last_command_msg->name;
    hw_positions_input_ = last_command_msg->position;
    hw_velocities_input_ = last_command_msg->velocity;
    joint_names_input_ = convertToRosElevatorPosition(joint_names_input_);

    for (auto i = 0u; i < joint_names_output_.size(); i++)
    {
      for (auto j = 0u; j < joint_names_input_.size(); j++)
      {
        if (strcmp(joint_names_input_[j].c_str(), info_.joints[i].name.c_str()) == 0)
        {
          hw_positions_input_[i] = convertToRosPosition(hw_positions_input_[j]);
          hw_velocities_input_[i] = convertToRosVelocity((double)hw_velocities_input_[j]);
          break;
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type swerve_hardware::RealDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    convertToRealPositions(hw_command_position_);
    convertToRealElevatorPosition();
    // Publish to Real
    if (realtime_real_publisher_->trylock())
    {
      auto &realtime_real_command_ = realtime_real_publisher_->msg_;
      realtime_real_command_.header.stamp = node_->get_clock()->now();
      realtime_real_command_.name = joint_names_output_;
      realtime_real_command_.velocity = hw_command_velocity_output_;
      realtime_real_command_.position = hw_command_position_output_;
      realtime_real_publisher_->unlockAndPublish();
    }
    rclcpp::spin_some(node_);

    return hardware_interface::return_type::OK;
  }

} // namespace swerve_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    swerve_hardware::RealDriveHardware, hardware_interface::SystemInterface)
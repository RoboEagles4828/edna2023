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

#include "swerve_hardware/test_drive.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "swerve_hardware/motion_magic.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

namespace swerve_hardware
{

hardware_interface::CallbackReturn TestDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  
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
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> TestDriveHardware::export_state_interfaces()
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



std::vector<hardware_interface::CommandInterface> TestDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    auto joint = info_.joints[i];
    RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Joint Name %s", joint.name.c_str());

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str() );
      joint_types_[i] = hardware_interface::HW_IF_VELOCITY;

      // Add the command interface with a pointer to i of vel commands
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));

      // Make i of the pos command interface 0.0
      hw_command_position_[i] = 0.0;

    } else {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Position Joint: %s", joint.name.c_str() );
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



hardware_interface::CallbackReturn TestDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Activating ...please wait...");
  // Set Default Values for State Interface Arrays
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_command_velocity_[i] = 0.0;
    hw_command_position_[i] = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn TestDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ||                        ||
// \/ THE STUFF THAT MATTERS \/

hardware_interface::return_type TestDriveHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  // Dumb Pass Through
  // If you give the command for x velocity then the state is x velocity

  // Loop through each joint name
  // Check if joint name is in velocity command map
  // If it is, use the index from the map to get the value in the velocity array
  // If velocity not in map, set velocity value to 0
  // Perform the same for position  

  double dt = period.seconds();
  for (auto i = 0u; i < joint_names_.size(); i++)
  {
    if (joint_types_[i] == hardware_interface::HW_IF_VELOCITY)
    {
      hw_velocities_[i] = hw_command_velocity_[i];
      hw_positions_[i] = hw_positions_[i] + dt * hw_velocities_[i];
    }
    else if (joint_types_[i] == hardware_interface::HW_IF_POSITION)
    {
      auto vel = motion_magic_[i].getNextVelocity(hw_command_position_[i], hw_positions_[i], hw_velocities_[i], dt);
      // RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Current: %f, Target: %f Vel: %f", hw_positions_[i], hw_command_position_[i], vel);
      hw_velocities_[i] = vel;
      hw_positions_[i] = hw_positions_[i] + hw_velocities_[i] * dt;

      // Test without any velocity smoothing
      // RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Cmd: %f Name: %s", hw_command_position_[i], joint_names_[i].c_str());
      // hw_velocities_[i] = 0.0;
      // hw_positions_[i] = hw_command_position_[i];
    }
  }
  
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type TestDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Do Nothing
  // Uncomment below if you want verbose messages for debugging.
  // for (auto i = 0u; i < hw_command_velocity_.size(); i++)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Wheel %u Velocity: %f", i, hw_command_velocity_[i]);
  // }

  // RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "[%f] Joint 2 Position: %f", time.seconds(), hw_command_position_[2]);

  return hardware_interface::return_type::OK;
}

}  // namespace swerve_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  swerve_hardware::TestDriveHardware, hardware_interface::SystemInterface)
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

  double RealDriveHardware::parse_double(const std::string & text)
  {
    return std::atof(text.c_str());
  }

  bool RealDriveHardware::parse_bool(const std::string & text)
  {
    if(strcmp(text.c_str(), "true") == 0) {
      return true;
    } else {
      return false;
    }
  }

  hardware_interface::CallbackReturn RealDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
  {

    node_ = rclcpp::Node::make_shared("isaac_hardware_interface");

    // PUBLISHER SETUP
    real_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_command_topic_, rclcpp::SystemDefaultsQoS());
    realtime_real_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        real_publisher_);
    
    real_arm_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_arm_command_topic_, rclcpp::SystemDefaultsQoS());
    realtime_real_arm_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        real_arm_publisher_);


    // SUBSCRIBER SETUP
    const sensor_msgs::msg::JointState empty_joint_state;
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    received_joint_msg_ptr_.set(std::make_shared<sensor_msgs::msg::JointState>(empty_joint_state));
    real_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_, qos,
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

    // GLOBAL VECTOR SETUP
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    hw_command_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_types_.resize(info_.joints.size(), "");

  
    // JOINT GROUPS
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      const auto & joint = info_.joints.at(i);

      bool use_percent = false;
      double max = parse_double(joint.command_interfaces[0].max);
      double min = parse_double(joint.command_interfaces[0].min);

      auto param_percent_it = joint.parameters.find("percent");
      if (param_percent_it != joint.parameters.end()) {
        use_percent = parse_bool(joint.parameters.at("percent"));
      }

      // Mimics
      if (joint.parameters.find("mimic") != joint.parameters.cend())
      {
        const auto mimicked_joint_it = std::find_if(
          info_.joints.begin(), info_.joints.end(),
          [&mimicked_joint =
            joint.parameters.at("mimic")](const hardware_interface::ComponentInfo & joint_info)
          { return joint_info.name == mimicked_joint; });
        if (mimicked_joint_it == info_.joints.cend())
        {
          throw std::runtime_error(
            std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
        }
        MimicJoint mimic_joint;
        mimic_joint.joint_index = i;
        mimic_joint.mimicked_joint_index = std::distance(info_.joints.begin(), mimicked_joint_it);
        
        // Multiplier and offset
        auto param_mult_it = joint.parameters.find("multiplier");
        if (param_mult_it != joint.parameters.end()) {
          mimic_joint.multiplier = parse_double(joint.parameters.at("multiplier"));
        }
        
        auto param_off_it = joint.parameters.find("offset");
        if (param_off_it != joint.parameters.end()) {
          mimic_joint.offset = parse_double(joint.parameters.at("offset"));
        }
        mimic_joints_.push_back(mimic_joint);
      }

      // 
      else if (joint.parameters.find("arm_group") != joint.parameters.cend())
      {
        JointGroupMember member;
        member.joint_index = i;
        member.joint_name = joint.name;
        member.percent = use_percent;
        member.max = max;
        member.min = min;

        arm_names_output_.push_back(joint.name);
        arm_joints_.push_back(member);
      } else {
        JointGroupMember member;
        member.joint_index = i;
        member.joint_name = joint.name;
        member.percent = use_percent;
        member.max = max;
        member.min = min;

        drive_names_output_.push_back(joint.name);
        drive_joints_.push_back(member);
      }
    }

    hw_command_arm_velocity_output_.resize(arm_joints_.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_arm_position_output_.resize(arm_joints_.size(), std::numeric_limits<double>::quiet_NaN());
    
    hw_command_drive_velocity_output_.resize(drive_joints_.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_drive_position_output_.resize(drive_joints_.size(), std::numeric_limits<double>::quiet_NaN());

    // Check that the info we were passed makes sense   
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
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
      if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str() );
        joint_types_[i] = hardware_interface::HW_IF_VELOCITY;

        // Add the command interface with a pointer to i of vel commands
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));

        // Make i of the pos command interface 0.0
        hw_command_position_[i] = 0.0;

      } else {
        RCLCPP_INFO(rclcpp::get_logger("RealDriveHardware"), "Added Position Joint: %s", joint.name.c_str() );
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

  hardware_interface::return_type RealDriveHardware::read(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
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

    // Match Arm and Drive Joints
    for (auto i = 0u; i < names.size(); i++) {
      for (const auto & arm_joint : arm_joints_)
      {
        if (strcmp(names[i].c_str(), arm_joint.joint_name.c_str()) == 0) {
          if (arm_joint.percent) {
            double scale = arm_joint.max - arm_joint.min;
            hw_positions_[arm_joint.joint_index] = convertToRosPosition(positions[i] * scale + arm_joint.min);
            hw_velocities_[arm_joint.joint_index] = convertToRosVelocity((float)velocities[i] * scale);
          } else {
            hw_positions_[arm_joint.joint_index] = convertToRosPosition(positions[i]);
            hw_velocities_[arm_joint.joint_index] = convertToRosVelocity((float)velocities[i]);
          }
          break;
        }
      }

      for (const auto & drive_joint : drive_joints_)
      {
        if (strcmp(names[i].c_str(), drive_joint.joint_name.c_str()) == 0) {
          hw_positions_[drive_joint.joint_index] = convertToRosPosition(positions[i]);
          hw_velocities_[drive_joint.joint_index] = convertToRosVelocity((float)velocities[i]);
          break;
        }
      }
    }

    // Apply Mimic Joints
    for (const auto & mimic_joint : mimic_joints_)
    {
      hw_positions_[mimic_joint.joint_index] = hw_positions_[mimic_joint.mimicked_joint_index] * mimic_joint.multiplier + mimic_joint.offset;
      hw_velocities_[mimic_joint.joint_index] = hw_velocities_[mimic_joint.mimicked_joint_index] * mimic_joint.multiplier;
    }
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type swerve_hardware::RealDriveHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // convertToRealPositions(hw_command_position_);
    // convertToRealElevatorPosition();
    // Publish to Real

    for (auto i = 0u; i < drive_joints_.size(); ++i)
    {
      auto joint = drive_joints_[i];
      hw_command_drive_velocity_output_[i] = hw_command_velocity_[joint.joint_index];
      hw_command_drive_position_output_[i] = hw_command_position_[joint.joint_index];
    }

    for (auto i = 0u; i < arm_joints_.size(); ++i)
    {
      auto joint = arm_joints_[i];
      hw_command_arm_velocity_output_[i] = hw_command_velocity_[joint.joint_index];
      hw_command_arm_position_output_[i] = hw_command_position_[joint.joint_index];
    }
  
    if (realtime_real_publisher_->trylock())
    {
      auto &realtime_real_command_ = realtime_real_publisher_->msg_;
      realtime_real_command_.header.stamp = node_->get_clock()->now();
      realtime_real_command_.name = drive_names_output_;
      realtime_real_command_.velocity = hw_command_drive_velocity_output_;
      realtime_real_command_.position = hw_command_drive_position_output_;
      realtime_real_publisher_->unlockAndPublish();
    }

    if (realtime_real_arm_publisher_->trylock())
    {
      auto &realtime_real_arm_command_ = realtime_real_arm_publisher_->msg_;
      realtime_real_arm_command_.header.stamp = node_->get_clock()->now();
      realtime_real_arm_command_.name = arm_names_output_;
      realtime_real_arm_command_.velocity = hw_command_arm_velocity_output_;
      realtime_real_arm_command_.position = hw_command_arm_position_output_;
      realtime_real_arm_publisher_->unlockAndPublish();
    }
    rclcpp::spin_some(node_);

    return hardware_interface::return_type::OK;
  }

} // namespace swerve_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    swerve_hardware::RealDriveHardware, hardware_interface::SystemInterface)
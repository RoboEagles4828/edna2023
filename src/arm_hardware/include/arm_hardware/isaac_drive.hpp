#ifndef ARM_HARDWARE__ISAAC_DRIVE_HPP_
#define ARM_HARDWARE__ISAAC_DRIVE_HPP_

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
#include "arm_hardware/visibility_control.h"

namespace arm_hardware
{
class IsaacArmDrive : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(IsaacArmDrive)

        ARM_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        
        ARM_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ARM_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ARM_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        ARM_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        ARM_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        ARM_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        std::vector<double> hw_command_position_;

        std::vector<double> hw_positions_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> joint_types_;
          double MAX_VELOCITY = 5.0;
        double MAX_ACCELERATION = 1.0;

        // Pub Sub to isaac
        std::string joint_state_topic_ = "isaac_arm_joint_states";
        std::string joint_command_topic_ = "isaac_arm_joint_commands";
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> isaac_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
            realtime_isaac_publisher_ = nullptr;

        bool subscriber_is_active_ = false;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr isaac_subscriber_ = nullptr;
        realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>> received_joint_msg_ptr_{nullptr};

        // Converts isaac position range -2pi - 2pi into expected ros position range -pi - pi
        double convertToRosPosition(double isaac_position);
};
}
#endif
// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "swerve_controller/swerve_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
  constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
  constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
  constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
} // namespace

namespace swerve_controller
{
  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;
  using lifecycle_msgs::msg::State;

  Wheel::Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name) : velocity_(velocity), name(std::move(name)) {}
  void Wheel::set_velocity(double velocity)
  {
    velocity_.get().set_value(velocity);
  }

  Axle::Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_position, std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_position, std::string name) : command_position_(command_position), state_position_(state_position), name(std::move(name)) {}
  void Axle::set_position(double position)
  {
    command_position_.get().set_value(position);
  }
  double Axle::get_position(void)
  {
    // temporary
    return state_position_.get().get_value();
  }

  void optimize(double& target, const double& current, double& wheel_velocity) {
    double target_copy = target;
    double diff = target_copy - current;
    // Check one way
    if (diff > M_PI) {
      target_copy -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
      target_copy += 2.0 * M_PI;
    }
    // Check other way
    diff = target_copy - current;
    if(std::abs(diff) > M_PI / 2.0) {
      // Get better position 180 degrees away
      if (target < 0.0) {
        target += M_PI;
      } else {
        target -= M_PI;
      }
      // Reverse direction
      wheel_velocity *= -1.0;
    }
  }


  SwerveController::SwerveController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn SwerveController::on_init()
  {
    try
    {
      // with the lifecycle node being initialized, we can declare parameters
      auto_declare<std::string>("front_left_wheel_joint", front_left_wheel_joint_name_);
      auto_declare<std::string>("front_right_wheel_joint", front_right_wheel_joint_name_);
      auto_declare<std::string>("rear_left_wheel_joint", rear_left_wheel_joint_name_);
      auto_declare<std::string>("rear_right_wheel_joint", rear_right_wheel_joint_name_);

      auto_declare<std::string>("front_left_axle_joint", front_left_axle_joint_name_);
      auto_declare<std::string>("front_right_axle_joint", front_right_axle_joint_name_);
      auto_declare<std::string>("rear_left_axle_joint", rear_left_axle_joint_name_);
      auto_declare<std::string>("rear_right_axle_joint", rear_right_axle_joint_name_);

      auto_declare<double>("chassis_length_meters", wheel_params_.x_offset);
      auto_declare<double>("chassis_width_meters", wheel_params_.y_offset);
      auto_declare<double>("wheel_radius_meters", wheel_params_.radius);
      auto_declare<double>("max_wheel_angular_velocity", max_wheel_angular_velocity_);

      auto_declare<double>("cmd_vel_timeout_seconds", cmd_vel_timeout_milliseconds_.count() / 1000.0);
      auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

      // Limits
      auto_declare<bool>("linear.x.has_velocity_limits", false);
      auto_declare<bool>("linear.x.has_acceleration_limits", false);
      auto_declare<bool>("linear.x.has_jerk_limits", false);
      auto_declare<double>("linear.x.max_velocity", 0.0);
      auto_declare<double>("linear.x.min_velocity", 0.0);
      auto_declare<double>("linear.x.max_acceleration", 0.0);
      auto_declare<double>("linear.x.min_acceleration", 0.0);
      auto_declare<double>("linear.x.max_jerk", 0.0);
      auto_declare<double>("linear.x.min_jerk", 0.0);

      auto_declare<bool>("linear.y.has_velocity_limits", false);
      auto_declare<bool>("linear.y.has_acceleration_limits", false);
      auto_declare<bool>("linear.y.has_jerk_limits", false);
      auto_declare<double>("linear.y.max_velocity", 0.0);
      auto_declare<double>("linear.y.min_velocity", 0.0);
      auto_declare<double>("linear.y.max_acceleration", 0.0);
      auto_declare<double>("linear.y.min_acceleration", 0.0);
      auto_declare<double>("linear.y.max_jerk", 0.0);
      auto_declare<double>("linear.y.min_jerk", 0.0);

      auto_declare<bool>("angular.z.has_velocity_limits", false);
      auto_declare<bool>("angular.z.has_acceleration_limits", false);
      auto_declare<bool>("angular.z.has_jerk_limits", false);
      auto_declare<double>("angular.z.max_velocity", 0.0);
      auto_declare<double>("angular.z.min_velocity", 0.0);
      auto_declare<double>("angular.z.max_acceleration", 0.0);
      auto_declare<double>("angular.z.min_acceleration", 0.0);
      auto_declare<double>("angular.z.max_jerk", 0.0);
      auto_declare<double>("angular.z.min_jerk", 0.0);

    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration SwerveController::command_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    conf_names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);

    conf_names.push_back(front_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(front_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration SwerveController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    conf_names.push_back(front_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(front_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::return_type SwerveController::update(
      const rclcpp::Time &time, const rclcpp::Duration & period)
  {
    auto logger = get_node()->get_logger();
    auto clk = * get_node()->get_clock();
    
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
      if (!is_halted)
      {
        halt();
        is_halted = true;
      }
      return controller_interface::return_type::OK;
    }

    const auto current_time = time;

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
      return controller_interface::return_type::ERROR;
    }

    // const auto age_of_last_command = current_time - last_command_msg->header.stamp;
    // // Brake if cmd_vel has timeout, override the stored command
    // if (age_of_last_command > cmd_vel_timeout_milliseconds_)
    // {
    //   halt();
    // }

    // INPUTS
    Twist command = *last_command_msg;
    auto & last_command = previous_commands_.back().twist;
    auto & second_to_last_command = previous_commands_.front().twist;
    double &linear_x_velocity_comand = command.twist.linear.x;
    double &linear_y_velocity_comand = command.twist.linear.y;
    double &angular_velocity_comand = command.twist.angular.z;
    
    double original_linear_x_velocity_comand = linear_x_velocity_comand;
    double original_linear_y_velocity_comand = linear_y_velocity_comand;
    double original_angular_velocity_comand = angular_velocity_comand;

    // Limits
    limiter_linear_X_.limit(linear_x_velocity_comand, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
    limiter_linear_Y_.limit(linear_y_velocity_comand, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
    limiter_angular_Z_.limit(angular_velocity_comand, last_command.angular.z, second_to_last_command.angular.z, period.seconds());
    
    previous_commands_.pop();
    previous_commands_.emplace(command);

    if (linear_x_velocity_comand != original_linear_x_velocity_comand) {
      RCLCPP_WARN_THROTTLE(logger, clk, 1000, "Limiting Linear X %f -> %f", original_linear_x_velocity_comand, linear_x_velocity_comand);
    }
    if (linear_y_velocity_comand != original_linear_y_velocity_comand) {
      RCLCPP_WARN_THROTTLE(logger, clk, 1000, "Limiting Linear Y %f -> %f", original_linear_y_velocity_comand, linear_y_velocity_comand);
    }
    if (angular_velocity_comand != original_angular_velocity_comand) {
      RCLCPP_WARN_THROTTLE(logger, clk, 1000, "Limiting Angular Z %f -> %f", original_angular_velocity_comand, angular_velocity_comand);
    }

    double x_offset = wheel_params_.x_offset;
    double radius = wheel_params_.radius;

    // get current wheel positions
    const double front_left_current_pos = front_left_axle_command_handle_->get_position();
    const double front_right_current_pos = front_right_axle_command_handle_->get_position();
    const double rear_left_current_pos = rear_left_axle_command_handle_->get_position();
    const double rear_right_current_pos = rear_right_axle_command_handle_->get_position();

    // Compute Wheel Velocities and Positions
    const double a = (linear_y_velocity_comand * -1.0) - angular_velocity_comand * x_offset / 2.0;
    const double b = (linear_y_velocity_comand * -1.0) + angular_velocity_comand * x_offset / 2.0;
    const double c = linear_x_velocity_comand - angular_velocity_comand * x_offset / 2.0;
    const double d = linear_x_velocity_comand + angular_velocity_comand * x_offset / 2.0;

    double front_left_velocity = sqrt(pow(b, 2) + pow(d, 2)) / radius;
    double front_right_velocity = sqrt(pow(b, 2) + pow(c, 2)) / radius;
    double rear_left_velocity = sqrt(pow(a, 2) + pow(d, 2)) / radius;
    double rear_right_velocity = sqrt(pow(a, 2) + pow(c, 2)) / radius;

    // Normalize wheel velocities if any are greater than max
    double velMax = std::max({front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity});
    if (velMax > max_wheel_angular_velocity_)
    {
      front_left_velocity = front_left_velocity/velMax * max_wheel_angular_velocity_;
      front_right_velocity = front_right_velocity/velMax * max_wheel_angular_velocity_;
      rear_left_velocity = rear_left_velocity/velMax * max_wheel_angular_velocity_;
      rear_right_velocity = rear_right_velocity/velMax * max_wheel_angular_velocity_;
    }

    double front_left_position;
    double front_right_position ;
    double rear_left_position;
    double rear_right_position;

    // Make position current if no movement is given
    if (std::abs(linear_x_velocity_comand) <= 0.1 && std::abs(linear_y_velocity_comand) <= 0.1 && std::abs(angular_velocity_comand) <= 0.1)
    {
      front_left_position = front_left_current_pos;
      front_right_position = front_right_current_pos;
      rear_left_position = rear_left_current_pos;
      rear_right_position = rear_right_current_pos;
    } else {
      front_left_position = atan2(b, d);
      front_right_position = atan2(b, c);
      rear_left_position = atan2(a, d);
      rear_right_position = atan2(a, c);

      // Optimization
      optimize(front_left_position, front_left_current_pos, front_left_velocity);
      optimize(front_right_position, front_right_current_pos, front_right_velocity);
      optimize(rear_left_position, rear_left_current_pos, rear_left_velocity);
      optimize(rear_right_position, rear_right_current_pos, rear_right_velocity);
    }

    // Old Limit Wheel Velocity
    // front_left_velocity=limiter_wheel_.limit(front_left_velocity,last_wheel_commands[0], second_last_wheel_commands[0],0.1);
    // front_right_velocity=limiter_wheel_.limit(front_right_velocity,last_wheel_commands[1], second_last_wheel_commands[1],0.1);
    // rear_left_velocity=limiter_wheel_.limit(rear_left_velocity,last_wheel_commands[2], second_last_wheel_commands[2],0.1);
    // rear_right_velocity=limiter_wheel_.limit(rear_right_velocity,last_wheel_commands[3], second_last_wheel_commands[3],0.1);
    // second_last_wheel_commands= last_wheel_commands;

    
    front_left_wheel_command_handle_->set_velocity(front_left_velocity);
    front_right_wheel_command_handle_->set_velocity(front_right_velocity);
    rear_left_wheel_command_handle_->set_velocity(rear_left_velocity);
    rear_right_wheel_command_handle_->set_velocity(rear_right_velocity);

    front_left_axle_command_handle_->set_position(front_left_position);
    front_right_axle_command_handle_->set_position(front_right_position);
    rear_left_axle_command_handle_->set_position(rear_left_position);
    rear_right_axle_command_handle_->set_position(rear_right_position);

    // Time update
    const auto update_dt = current_time - previous_update_timestamp_;
    previous_update_timestamp_ = current_time;

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto logger = get_node()->get_logger();

    

    limiter_linear_X_ = SpeedLimiter(
      get_node()->get_parameter("linear.x.has_velocity_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_jerk_limits").as_bool(),
      get_node()->get_parameter("linear.x.min_velocity").as_double(),
      get_node()->get_parameter("linear.x.max_velocity").as_double(),
      get_node()->get_parameter("linear.x.min_acceleration").as_double(),
      get_node()->get_parameter("linear.x.max_acceleration").as_double(),
      get_node()->get_parameter("linear.x.min_jerk").as_double(),
      get_node()->get_parameter("linear.x.max_jerk").as_double());

    RCLCPP_WARN(logger, "Linear X Limiter Velocity Limits:      %f, %f", limiter_linear_X_.min_velocity_, limiter_linear_X_.max_velocity_);
    RCLCPP_WARN(logger, "Linear X Limiter Acceleration Limits:  %f, %f", limiter_linear_X_.min_acceleration_, limiter_linear_X_.max_acceleration_);
    RCLCPP_WARN(logger, "Linear X Limiter Jert Limits:          %f, %f", limiter_linear_X_.min_jerk_, limiter_linear_X_.max_jerk_);
    
    limiter_linear_Y_ = SpeedLimiter(
      get_node()->get_parameter("linear.y.has_velocity_limits").as_bool(),
      get_node()->get_parameter("linear.y.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("linear.y.has_jerk_limits").as_bool(),
      get_node()->get_parameter("linear.y.min_velocity").as_double(),
      get_node()->get_parameter("linear.y.max_velocity").as_double(),
      get_node()->get_parameter("linear.y.min_acceleration").as_double(),
      get_node()->get_parameter("linear.y.max_acceleration").as_double(),
      get_node()->get_parameter("linear.y.min_jerk").as_double(),
      get_node()->get_parameter("linear.y.max_jerk").as_double());

    RCLCPP_WARN(logger, "Linear Y Limiter Velocity Limits:      %f, %f", limiter_linear_Y_.min_velocity_, limiter_linear_Y_.max_velocity_);
    RCLCPP_WARN(logger, "Linear Y Limiter Acceleration Limits:  %f, %f", limiter_linear_Y_.min_acceleration_, limiter_linear_Y_.max_acceleration_);
    RCLCPP_WARN(logger, "Linear Y Limiter Jert Limits:          %f, %f", limiter_linear_Y_.min_jerk_, limiter_linear_Y_.max_jerk_);
    
    limiter_angular_Z_ = SpeedLimiter(
      get_node()->get_parameter("angular.z.has_velocity_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_jerk_limits").as_bool(),
      get_node()->get_parameter("angular.z.min_velocity").as_double(),
      get_node()->get_parameter("angular.z.max_velocity").as_double(),
      get_node()->get_parameter("angular.z.min_acceleration").as_double(),
      get_node()->get_parameter("angular.z.max_acceleration").as_double(),
      get_node()->get_parameter("angular.z.min_jerk").as_double(),
      get_node()->get_parameter("angular.z.max_jerk").as_double());
    
    RCLCPP_WARN(logger, "Angular Z Limiter Velocity Limits:      %f, %f", limiter_angular_Z_.min_velocity_, limiter_angular_Z_.max_velocity_);
    RCLCPP_WARN(logger, "Angular Z Limiter Acceleration Limits:  %f, %f", limiter_angular_Z_.min_acceleration_, limiter_angular_Z_.max_acceleration_);
    RCLCPP_WARN(logger, "Angular Z Limiter Jert Limits:          %f, %f", limiter_angular_Z_.min_jerk_, limiter_angular_Z_.max_jerk_);
    
    // Get Parameters
    front_left_wheel_joint_name_ = get_node()->get_parameter("front_left_wheel_joint").as_string();
    front_right_wheel_joint_name_ = get_node()->get_parameter("front_right_wheel_joint").as_string();
    rear_left_wheel_joint_name_ = get_node()->get_parameter("rear_left_wheel_joint").as_string();
    rear_right_wheel_joint_name_ = get_node()->get_parameter("rear_right_wheel_joint").as_string();

    front_left_axle_joint_name_ = get_node()->get_parameter("front_left_axle_joint").as_string();
    front_right_axle_joint_name_ = get_node()->get_parameter("front_right_axle_joint").as_string();
    rear_left_axle_joint_name_ = get_node()->get_parameter("rear_left_axle_joint").as_string();
    rear_right_axle_joint_name_ = get_node()->get_parameter("rear_right_axle_joint").as_string();

    if (front_left_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_left_wheel_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (front_right_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_right_wheel_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (rear_left_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_wheel_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (rear_right_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_wheel_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (front_left_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_left_axle_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (front_right_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_right_axle_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (rear_left_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_axle_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (rear_right_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_axle_joint_name is not set");
      return controller_interface::CallbackReturn::ERROR;
    }

    wheel_params_.x_offset = get_node()->get_parameter("chassis_length_meters").as_double();
    wheel_params_.y_offset = get_node()->get_parameter("chassis_width_meters").as_double();
    wheel_params_.radius = get_node()->get_parameter("wheel_radius_meters").as_double();
    max_wheel_angular_velocity_ = get_node()->get_parameter("max_wheel_angular_velocity").as_double();

    cmd_vel_timeout_milliseconds_ = std::chrono::milliseconds{
        static_cast<int>(get_node()->get_parameter("cmd_vel_timeout_seconds").as_double() * 1000.0)};
    use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

    // Run reset to make sure everything is initialized correctly
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    // initialize command subscriber
    if (use_stamped_vel_)
    {
      velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
          DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<Twist> msg) -> void
          {
            if (!subscriber_is_active_)
            {
              RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
              return;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
              RCLCPP_WARN_ONCE(
                  get_node()->get_logger(),
                  "Received TwistStamped with zero timestamp, setting it to current "
                  "time, this message will only be shown once");
              msg->header.stamp = get_node()->get_clock()->now();
            }
            received_velocity_msg_ptr_.set(std::move(msg));
          });
    }
    else
    {
      velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
          DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
          {
            if (!subscriber_is_active_)
            {
              RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
              return;
            }

            // Write fake header in the stored stamped command
            std::shared_ptr<Twist> twist_stamped;
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = get_node()->get_clock()->now();
          });
    }

    previous_update_timestamp_ = get_node()->get_clock()->now();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
  {
    front_left_wheel_command_handle_ = get_wheel(front_left_wheel_joint_name_);
    front_right_wheel_command_handle_ = get_wheel(front_right_wheel_joint_name_);
    rear_left_wheel_command_handle_ = get_wheel(rear_left_wheel_joint_name_);
    rear_right_wheel_command_handle_ = get_wheel(rear_right_wheel_joint_name_);
    front_left_axle_command_handle_ = get_axle(front_left_axle_joint_name_);
    front_right_axle_command_handle_ = get_axle(front_right_axle_joint_name_);
    rear_left_axle_command_handle_ = get_axle(rear_left_axle_joint_name_);
    rear_right_axle_command_handle_ = get_axle(rear_right_axle_joint_name_);

    if (!front_left_wheel_command_handle_ || !front_right_wheel_command_handle_ || !rear_left_wheel_command_handle_ || !rear_right_wheel_command_handle_ || !front_left_axle_command_handle_ || !front_right_axle_command_handle_ || !rear_left_axle_command_handle_ || !rear_right_axle_command_handle_)
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    subscriber_is_active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool SwerveController::reset()
  {
    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    std::queue<Twist> empty;
    std::swap(previous_commands_, empty);

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    return true;
  }

  controller_interface::CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void SwerveController::halt()
  {
    front_left_wheel_command_handle_->set_velocity(0.0);
    front_right_wheel_command_handle_->set_velocity(0.0);
    rear_left_wheel_command_handle_->set_velocity(0.0);
    rear_right_wheel_command_handle_->set_velocity(0.0);
    auto logger = get_node()->get_logger();
    RCLCPP_WARN(logger, "-----HALT CALLED : STOPPING ALL MOTORS-----");
  }

  std::shared_ptr<Wheel> SwerveController::get_wheel(const std::string &wheel_name)
  {
    auto logger = get_node()->get_logger();
    if (wheel_name.empty())
    {
      RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
      return nullptr;
    }

    // Get Command Handle for joint
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wheel_name](const auto &interface)
        {
          return interface.get_name() == wheel_name + "/velocity" &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return nullptr;
    }
    auto cmd_interface_name = command_handle->get_name();
    RCLCPP_INFO(logger, "FOUND! wheel cmd interface %s", cmd_interface_name.c_str());
    return std::make_shared<Wheel>(std::ref(*command_handle), wheel_name);
  }
  std::shared_ptr<Axle> SwerveController::get_axle(const std::string &axle_name)
  {
    auto logger = get_node()->get_logger();
    if (axle_name.empty())
    {
      RCLCPP_ERROR(logger, "Axle joint name not given. Make sure all joints are specified.");
      return nullptr;
    }

    // Get Command Handle for joint
    const auto command_handle_position = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&axle_name](const auto &interface)
        {
          return interface.get_name() == axle_name + "/position" &&
                 interface.get_interface_name() == HW_IF_POSITION;
        });
    const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&axle_name](const auto &interface)
        {
          return interface.get_name() == axle_name + "/position" &&
                 interface.get_interface_name() == HW_IF_POSITION;
        });
    if (command_handle_position == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", axle_name.c_str());
      return nullptr;
    }
    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", axle_name.c_str());
      return nullptr;
    }
    auto cmd_interface_name = command_handle_position->get_name();
    RCLCPP_INFO(logger, "FOUND! axle cmd interface %s", cmd_interface_name.c_str());
    return std::make_shared<Axle>(std::ref(*command_handle_position), std::ref(*state_handle), axle_name);
  }

} // namespace swerve_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    swerve_controller::SwerveController, controller_interface::ControllerInterface)

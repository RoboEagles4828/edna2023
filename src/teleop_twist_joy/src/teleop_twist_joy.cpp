/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include "teleop_twist_joy/teleop_twist_joy.hpp"
#include "edna_interfaces/srv/set_bool.hpp"
#include <functional> // for bind()
using namespace std;

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
   * directly into base nodes.
   */
  struct TeleopTwistJoy::Impl
  {
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr &, const std::string &which_map);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void timerCallback();
    void resetOrientationCallback(const std::shared_ptr<edna_interfaces::srv::SetBool::Request> request, std::shared_ptr<edna_interfaces::srv::SetBool::Response> response);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Service<edna_interfaces::srv::SetBool>::SharedPtr reset_orientation_service;
    rclcpp::TimerBase::SharedPtr timer_callback_;
    rclcpp::Client<edna_interfaces::srv::SetBool>::SharedPtr start_writer_client_;
    sensor_msgs::msg::Imu::SharedPtr last_msg;

    bool require_enable_button;
    int64_t enable_button;
    int64_t enable_turbo_button;
    int64_t enable_field_oriented_button;
    int64_t start_writer_button;

    int fieldOrientationButtonLastState = 0;
    int turboButtonLastState = 0;
    double last_offset = 0.0;
    double rotation_offset = 0.0;
    bool fieldOrientationEnabled = true;
    bool turboEnabled = false;
    int serviceButtonLastState = 0;
    bool serviceEnabled = false;
    std::map<std::string, int64_t> axis_linear_map;
    std::map<std::string, std::map<std::string, double>> scale_linear_map;

    std::map<std::string, int64_t> axis_angular_map;
    std::map<std::string, std::map<std::string, double>> scale_angular_map;

    bool sent_disable_msg;
  };

  /**
   * Constructs TeleopTwistJoy.
   */
  TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions &options) : Node("teleop_twist_joy_node", options)
  {
    pimpl_ = new Impl;
    // rclcpp::Node node = Node("teleop_twist_joy_node", options); sensor_msgs/msg/Imu
    pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pimpl_->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("zed/imu/data", rclcpp::QoS(10).best_effort(),
                                                                          std::bind(&TeleopTwistJoy::Impl::imuCallback, this->pimpl_, std::placeholders::_1));
    pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10).best_effort(),
                                                                       std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));
    // pimpl_->client = this->create_client<writer_srv::srv::StartWriter>("start_writer");
    // pimpl_->timer_callback_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&TeleopTwistJoy::Impl::timerCallback,this->pimpl_));

    // pimpl_->client = create_client<writer_srv::srv::StartWriter>("start_writer",
    //                             [this](std::shared_ptr<writer_srv::srv::StartWriter::Request> /*request*/,  // NOLINT
    //                                    std::shared_ptr<writer_srv::srv::StartWriter::Response> response) {  // NOLINT
    //                               return startServiceCallback(std::move(response));     // NOLINT
    pimpl_->reset_orientation_service = create_service<edna_interfaces::srv::SetBool>("reset_field_oriented", std::bind(&TeleopTwistJoy::Impl::resetOrientationCallback, this->pimpl_, std::placeholders::_1, std::placeholders::_2));

    //                             });
    pimpl_->start_writer_client_ = create_client<edna_interfaces::srv::SetBool>("set_bool");

    pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

    pimpl_->enable_button = this->declare_parameter("enable_button", 5);

    pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);
    pimpl_->enable_field_oriented_button = this->declare_parameter("enable_field_oriented_button", 8);
    pimpl_->start_writer_button = this->declare_parameter("start_writer_button", 6);
    this->declare_parameter("offset", 0.0);
    pimpl_->last_offset = this->get_parameter("offset").as_double();

    std::map<std::string, int64_t> default_linear_map{
        {"x", 5L},
        {"y", -1L},
        {"z", -1L},
    };
    this->declare_parameters("axis_linear", default_linear_map);
    this->get_parameters("axis_linear", pimpl_->axis_linear_map);

    std::map<std::string, int64_t> default_angular_map{
        {"yaw", 2L},
        {"pitch", -1L},
        {"roll", -1L},
    };
    this->declare_parameters("axis_angular", default_angular_map);
    this->get_parameters("axis_angular", pimpl_->axis_angular_map);

    std::map<std::string, double> default_scale_linear_normal_map{
        {"x", 0.5},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear", default_scale_linear_normal_map);
    this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

    std::map<std::string, double> default_scale_linear_turbo_map{
        {"x", 1.0},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
    this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

    std::map<std::string, double> default_scale_angular_normal_map{
        {"yaw", 0.5},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("scale_angular", default_scale_angular_normal_map);
    this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

    std::map<std::string, double> default_scale_angular_turbo_map{
        {"yaw", 1.0},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
    this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

    ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "TeleopTwistJoy",
                        "Teleop enable button %" PRId64 ".", pimpl_->enable_button);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
                        "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);

    for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
         it != pimpl_->axis_linear_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
                          it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
                          "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
    }

    for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
         it != pimpl_->axis_angular_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
                          it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
                          "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
    }

    pimpl_->sent_disable_msg = false;

    auto param_callback =
        [this](std::vector<rclcpp::Parameter> parameters)
    {
      static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                                "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                                "enable_button", "enable_turbo_button", "enable_field_oriented_button", "start_writer_button", "offset"};
      static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                   "scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z",
                                                   "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll",
                                                   "scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll"};
      static std::set<std::string> boolparams = {"require_enable_button"};
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      // Loop to check if changed parameters are of expected data type
      for (const auto &parameter : parameters)
      {
        if (intparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
          {
            result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
        else if (doubleparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
        else if (boolparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
          {
            result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
      }

      // Loop to assign changed parameters to the member variables
      for (const auto &parameter : parameters)
      {

        if (parameter.get_name() == "require_enable_button")
        {
          this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        if (parameter.get_name() == "enable_button")
        {
          this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "enable_turbo_button")
        {
          this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "enable_field_oriented_button")
        {
          this->pimpl_->enable_field_oriented_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "start_writer_button")
        {
          this->pimpl_->start_writer_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.x")
        {
          this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.y")
        {
          this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.z")
        {
          this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.yaw")
        {
          this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.pitch")
        {
          this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.roll")
        {
          this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.x")
        {
          this->pimpl_->scale_linear_map["turbo"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.y")
        {
          this->pimpl_->scale_linear_map["turbo"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.z")
        {
          this->pimpl_->scale_linear_map["turbo"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.x")
        {
          this->pimpl_->scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.y")
        {
          this->pimpl_->scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.z")
        {
          this->pimpl_->scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.yaw")
        {
          this->pimpl_->scale_angular_map["turbo"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.pitch")
        {
          this->pimpl_->scale_angular_map["turbo"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.roll")
        {
          this->pimpl_->scale_angular_map["turbo"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.yaw")
        {
          this->pimpl_->scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.pitch")
        {
          this->pimpl_->scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.roll")
        {
          this->pimpl_->scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
      }
      return result;
    };

    callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  TeleopTwistJoy::~TeleopTwistJoy()
  {
    delete pimpl_;
  }

  double getVal(sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> &axis_map,
                const std::map<std::string, double> &scale_map, const std::string &fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        axis_map.at(fieldname) == -1L ||
        scale_map.find(fieldname) == scale_map.end() ||
        static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
  }
  double get_scale_val(const std::map<std::string, int64_t> &axis_map,
                       const std::map<std::string, double> &scale_map, const std::string &fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        axis_map.at(fieldname) == -1L ||
        scale_map.find(fieldname) == scale_map.end())
    {
      return 0.0;
    }

    return scale_map.at(fieldname);
  }
  double get_orientation_val(sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    if (!imu_msg)
    {
      return 0.0;
    }
    double x = imu_msg->orientation.x;
    double y = imu_msg->orientation.y;
    double z = imu_msg->orientation.z;
    double w = imu_msg->orientation.w;

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double angle = std::atan2(siny_cosp, cosy_cosp);

    return angle;
  }
  double correct_joystick_pos(const std::map<std::string, double> &scale_map, const std::string &fieldname, double lin_x_vel, double lin_y_vel)
  {
    if (sqrt(pow(lin_x_vel, 2) + pow(lin_y_vel, 2)) > 1)
    {
      double scale = scale_map.at(fieldname);
      if (scale < 0.001)
      {
        scale *= 10000;
      }
      if (fieldname == "x")
      {
        double vel_to_correct = sin(atan2(lin_x_vel, lin_y_vel)) * scale;
        return vel_to_correct;
      }
      else if (fieldname == "y")
      {
        double vel_to_correct = cos(atan2(lin_x_vel, lin_y_vel)) * scale;
        return vel_to_correct;
      }
    }
    else
    {
      if (fieldname == "x")
      {
        double vel_to_correct = sin(atan2(lin_x_vel, lin_y_vel)) * sqrt(pow(lin_x_vel, 2) + pow(lin_y_vel, 2));
        return vel_to_correct;
      }
      else if (fieldname == "y")
      {
        double vel_to_correct = cos(atan2(lin_x_vel, lin_y_vel)) * sqrt(pow(lin_x_vel, 2) + pow(lin_y_vel, 2));
        return vel_to_correct;
      }
    }

    return 0.0;
  }
  // void TeleopTwistJoy::startServiceCallBack(const std::shared_ptr<edna_interfaces::srv::SetBool::Response> response)
  // {
  //   if(joy){

  //   }
  // }
  void TeleopTwistJoy::Impl::timerCallback()
  {
    // it's good to firstly check if the service server is even ready to be called

    if (start_writer_client_->service_is_ready() && serviceEnabled && serviceButtonLastState == 1)
    {
      auto request = std::make_shared<edna_interfaces::srv::SetBool::Request>();
      request->data = true;
      while (!start_writer_client_->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("teleop_twist_joy"), "Interrupted while waiting for the service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("teleop_twist_joy"), "service not available, waiting again...");
      }
      auto result = start_writer_client_->async_send_request(request);
    }
    else if (start_writer_client_->service_is_ready() && !serviceEnabled && serviceButtonLastState == 1)
    {
      auto request = std::make_shared<edna_interfaces::srv::SetBool::Request>();
      request->data = false;
      while (!start_writer_client_->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("teleop_twist_joy"), "Interrupted while waiting for the service. Exiting.");
          break;
        }
        RCLCPP_INFO(rclcpp::get_logger("teleop_twist_joy"), "service not available, waiting again...");
      }

      auto result = start_writer_client_->async_send_request(request);

      // RCLCPP_INFO(rclcpp::get_logger("teleop_twist_joy"), "Bag recording stopped: %d", !result.get()->recording);
      // RCLCPP_INFO(rclcpp::get_logger("teleop_twist_joy"), "Path of Bag: %s", result.get()->path.c_str());
    }

    else if (!start_writer_client_->service_is_ready())
      RCLCPP_WARN(rclcpp::get_logger("teleop_twist_joy"), "[ServiceClientExample]: not calling service using callback, service not ready!");
  }
  void TeleopTwistJoy::Impl::resetOrientationCallback(const std::shared_ptr<edna_interfaces::srv::SetBool::Request> request, std::shared_ptr<edna_interfaces::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "received service call: %d", request->data);
    if (request->data)
    {
      rotation_offset=3.1415;
    }
    response->message = "succeeded";
    response->success = true;
  }

  void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr &joy_msg,
                                           const std::string &which_map)
  {
    // Initializes with zeros by default.

    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    double lin_x_vel = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
    double lin_y_vel = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
    double ang_z_vel = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
    double temp = correct_joystick_pos(scale_linear_map[which_map], "x", lin_x_vel, lin_y_vel);
    lin_y_vel = correct_joystick_pos(scale_linear_map[which_map], "y", lin_x_vel, lin_y_vel);
    lin_x_vel = temp;
    // RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "%f",lin_x_vel);
    // for( uint i =0u; i<joy_msg->buttons.size(); i++){
    //     RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "%d:%d:%ld",joy_msg->buttons[i],i,enable_field_oriented_button);
    // }

    if (enable_field_oriented_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > enable_field_oriented_button)
    {
      auto state = joy_msg->buttons[enable_field_oriented_button];
      if (state == 1 && fieldOrientationButtonLastState == 0)
      {
        fieldOrientationEnabled = !fieldOrientationEnabled;
        RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "Field Oriented: %d", fieldOrientationEnabled);
      }
      fieldOrientationButtonLastState = state;
    }
    if (start_writer_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > start_writer_button)
    {
      auto state = joy_msg->buttons[start_writer_button];
      if (state == 1 && serviceButtonLastState == 0)
      {
        serviceEnabled = !serviceEnabled;
        RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "Writer State: %d", serviceEnabled);
      }
      serviceButtonLastState = state;
    }
    // RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "robot_orientation: %f",last_offset);
    // Math for field oriented drive
    if (fieldOrientationEnabled)
    {
      double robot_imu_orientation = (get_orientation_val(last_msg));
      robot_imu_orientation += (ang_z_vel * last_offset) + rotation_offset;
      // RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "robot_orientation: %f", robot_imu_orientation);
      double temp = lin_x_vel * cos(robot_imu_orientation) + lin_y_vel * sin(robot_imu_orientation);
      lin_y_vel = -1 * lin_x_vel * sin(robot_imu_orientation) + lin_y_vel * cos(robot_imu_orientation);
      lin_x_vel = temp;
    }

    // Set Velocities in twist msg and publish
    cmd_vel_msg->linear.x = lin_x_vel;
    cmd_vel_msg->linear.y = lin_y_vel;
    cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
    cmd_vel_msg->angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
    cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
    cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
    cmd_vel_pub->publish(std::move(cmd_vel_msg));
    sent_disable_msg = false;
  }

  void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (enable_turbo_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button)
    {
      auto state = joy_msg->buttons[enable_turbo_button];
      if (state == 1 && turboButtonLastState == 0)
      {
        turboEnabled = !turboEnabled;
        RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "Turbo: %d", turboEnabled);
      }
      turboButtonLastState = state;
    }

    if (turboEnabled)
    {
      sendCmdVelMsg(joy_msg, "turbo");
    }
    else if (!require_enable_button ||
             (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
              joy_msg->buttons[enable_button]))
    {
      sendCmdVelMsg(joy_msg, "normal");
    }
    else
    {
      // When enable button is released, immediately send a single no-motion command
      // in order to stop the robot.
      if (!sent_disable_msg)
      {
        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel_pub->publish(std::move(cmd_vel_msg));
        sent_disable_msg = true;
      }
    }
  }
  void TeleopTwistJoy::Impl::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    // Saves current message as global pointer
    last_msg = imu_msg;
  }

} // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)

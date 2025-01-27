// Copyright 2025 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <iostream>

#include "trossen_slate/trossen_slate.hpp"

namespace trossen_slate
{

#ifdef ROS2
TrossenSlate::TrossenSlate(const rclcpp::NodeOptions & options)
: rclcpp::Node("trossen_slate", "", options), sys_cmd_{0},
  publish_tf_(false), current_time_(get_clock()->now()), tf_broadcaster_odom_(this)
{
  using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

  declare_parameter<bool>("publish_tf", false);
  declare_parameter<std::string>("odom_frame_name", "odom");
  declare_parameter<std::string>("base_frame_name", "base_link");

  get_parameter("publish_tf", publish_tf_);
  get_parameter("odom_frame_name", odom_frame_name_);
  get_parameter("base_frame_name", base_frame_name_);

  pub_odom_ = create_publisher<Odometry>("odom", 50);
  pub_battery_state_ = create_publisher<BatteryState>("battery_state", 1);

  sub_cmd_vel_ = create_subscription<Twist>(
    "cmd_vel",
    1,
    std::bind(&TrossenSlate::cmd_vel_callback, this, _1));

  srv_set_text_ = create_service<SetString>(
    "set_text",
    std::bind(&TrossenSlate::set_text_callback, this, _1, _2, _3));

  srv_motor_torque_status_ = create_service<SetBool>(
    "set_motor_torque_status",
    std::bind(&TrossenSlate::motor_torque_status_callback, this, _1, _2, _3));

  srv_enable_charging_ = create_service<SetBool>(
    "enable_charging",
    std::bind(&TrossenSlate::enable_charging_callback, this, _1, _2, _3));

  srv_set_light_state_ = create_service<SetLightState>(
    "set_light_state",
    std::bind(&TrossenSlate::set_light_state_callback, this, _1, _2, _3));

  std::string dev;
  if (!base_driver::chassisInit(dev)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize base port.");
    ::exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(get_logger(), "Initalized base at port '%s'.", dev.c_str());
  char version[32] = {0};
  if (base_driver::getVersion(version)) {
    RCLCPP_INFO(get_logger(), "Base version: %s", version);
  }
}

void TrossenSlate::update()
{
  rclcpp::spin_some(get_node_base_interface());
  current_time_ = get_clock()->now();

  if (!base_driver::updateChassisInfo(&data_)) {
    return;
  }

  // Update battery state every 10 iterations
  cnt_++;
  auto battery_state = BatteryState();
  if (cnt_ % 10 == 0) {
    battery_state.header.stamp = current_time_;
    battery_state.voltage = data_.voltage;
    battery_state.current = data_.current;
    battery_state.percentage = data_.charge;
    battery_state.power_supply_status = data_.system_state;
    pub_battery_state_->publish(battery_state);
  }

  if (is_first_odom_) {
    pose_[0] = data_.odom_x;
    pose_[1] = data_.odom_y;
    pose_[2] = data_.odom_z;
    is_first_odom_ = false;
  }

  // Create transform
  tf2::Quaternion q;
  q.setRPY(0, 0, wrap_angle(data_.odom_z - pose_[2]));
  auto odom_quat = tf2::toMsg(q);
  auto odom_trans = TransformStamped();
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = base_frame_name_;

  odom_trans.transform.translation.x =
    cos(-pose_[2]) * (data_.odom_x - pose_[0]) - sin(-pose_[2]) * (data_.odom_y - pose_[1]);
  odom_trans.transform.translation.y =
    sin(-pose_[2]) * (data_.odom_x - pose_[0]) + cos(-pose_[2]) * (data_.odom_y - -pose_[1]);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // Send the transform
  if (publish_tf_) {
    tf_broadcaster_odom_.sendTransform(odom_trans);
  }

  // Publish odometry
  auto odom = Odometry();
  odom.header.stamp = current_time_;
  odom.header.frame_id = odom_frame_name_;

  // Set pose
  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance[0] = (data_.system_state == SystemState::SYS_ESTOP) ? -1 : 1;

  // Set cmd velocity
  odom.child_frame_id = base_frame_name_;
  odom.twist.twist.linear.x = data_.vel_x;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = data_.vel_z;

  pub_odom_->publish(odom);
}

void TrossenSlate::cmd_vel_callback(const Twist::SharedPtr msg)
{
  data_.cmd_vel_x = msg->linear.x;
  data_.cmd_vel_z = msg->linear.z;
}

bool TrossenSlate::set_text_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetString::Request> req,
  const std::shared_ptr<SetString::Response> res)
{
  res->success = this->set_text(req->data);
  res->message = "Set base screen text to: '" + req->data + "'.";
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return true;
}

bool TrossenSlate::motor_torque_status_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  std::string result;
  res->success = this->enable_motor_torque(req->data, result);
  res->message = result;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return res->success;
}

bool TrossenSlate::enable_charging_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  std::string result;
  res->success = this->enable_charging(req->data, result);
  res->message = result;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return res->success;
}

bool TrossenSlate::set_light_state_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetLightState::Request> req,
  const std::shared_ptr<SetLightState::Response> res)
{
  // Cast the light_state from the request to the LightState enum class
  LightState light_state = static_cast<LightState>(req->light_state);

  // Call the set_light_state method with the casted light_state
  res->success = this->set_light_state(light_state);
  res->message = "Set light state to: " + std::to_string(req->light_state);
  RCLCPP_INFO(get_logger(), res->message.c_str());
  return true;
}

float TrossenSlate::wrap_angle(float angle)
{
  if (angle > M_PI) {
    angle = angle - 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle = angle + 2.0 * M_PI;
  }
  return angle;
}

#endif
TrossenSlate::TrossenSlate()
: sys_cmd_{0}
{
}

void TrossenSlate::read(base_driver::ChassisData & data)
{
  data = data_;
}

bool TrossenSlate::write(base_driver::ChassisData data)
{
  if (!base_driver::updateChassisInfo(&data)) {
    return false;
  }
  data_ = data;
  return true;
}

bool TrossenSlate::init_base(std::string & result)
{
  if (!base_initialized_) {
    std::string dev;
    if (!base_driver::chassisInit(dev)) {
      result = "Failed to initialize base port.";
      return false;
    } else {
      result = "Initalized base at port " + dev;
      char version[32] = {0};
      if (base_driver::getVersion(version)) {
        result += "\nBase version: " + std::string(version);
        base_initialized_ = true;
      }
    }
  } else {
    result = "Base already initialized.";
  }
  return true;
}

bool TrossenSlate::set_cmd_vel(float linear_vel, float angular_vel)
{
  linear_vel = std::min(MAX_VEL_X, std::max(-MAX_VEL_X, linear_vel));
  angular_vel = std::min(MAX_VEL_Z, std::max(-MAX_VEL_Z, angular_vel));

  data_.cmd_vel_x = linear_vel;
  data_.cmd_vel_z = angular_vel;

  return write(data_);
}

bool TrossenSlate::set_text(std::string text)
{
  base_driver::setText(text.c_str());
  return true;
}

bool TrossenSlate::set_light_state(LightState light_state)
{
  data_.light_state = static_cast<uint32_t>(light_state);
  return write(data_);
}

bool TrossenSlate::enable_motor_torque(bool enable, std::string & result)
{
  enable ? sys_cmd_ &= ~(1) : sys_cmd_ |= 1;
  bool success = base_driver::setSysCmd(sys_cmd_);
  std::string enabled_disabled = enable ? "enable" : "disable";

  if (success) {
    result = "Successfully " + enabled_disabled + "d motor torque.";
  } else {
    result = "Failed to " + enabled_disabled + " motor torque.";
  }
  return success;
}

bool TrossenSlate::enable_charging(bool enable, std::string & result)
{
  enable ? sys_cmd_ &= ~(2) : sys_cmd_ |= 2;
  bool success = base_driver::setSysCmd(sys_cmd_);

  std::string enabled_disabled = enable ? "enable" : "disable";

  if (success) {
    result = "Successfully " + enabled_disabled + "d charging.";
  } else {
    result = "Failed to " + enabled_disabled + " charging.";
  }
  return success;
}

std::array<float, 2> TrossenSlate::get_vel()
{
  std::array<float, 2> cmd_vel;
  cmd_vel[0] = data_.vel_x;
  cmd_vel[1] = data_.vel_z;
  return cmd_vel;
}

std::array<float, 3> TrossenSlate::get_pose()
{
  std::array<float, 3> pose;
  pose[0] = data_.odom_x;
  pose[1] = data_.odom_y;
  pose[2] = data_.odom_z;
  return pose;
}

uint32_t TrossenSlate::get_charge()
{
  return data_.charge;
}

float TrossenSlate::get_current()
{
  return data_.current;
}

float TrossenSlate::get_voltage()
{
  return data_.voltage;
}

} // namespace trossen_slate

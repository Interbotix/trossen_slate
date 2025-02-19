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

#include "trossen_slate/trossen_slate.hpp"

namespace trossen_slate
{

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

bool TrossenSlate::update_state()
{
  return base_driver::updateChassisInfo(&data_);
}

bool TrossenSlate::init_base(std::string & result)
{
  if (!base_initialized_) {
    result = "Using Trossen SLATE Driver Version: 'v" + std::to_string(VERSION_MAJOR) + "." +
      std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_PATCH) + "'.";
    std::string dev;
    if (!base_driver::chassisInit(dev)) {
      result += "\nFailed to initialize base port.";
      return false;
    } else {
      result += "\nInitalized base at port: '" + dev + "'.";
      char version[32] = {0};
      if (base_driver::getVersion(version)) {
        result += "\nBase version: 'v" + std::string(version) + "'.";
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
  std::array<float, 2> vel;
  vel[0] = data_.vel_x;
  vel[1] = data_.vel_z;
  return vel;
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

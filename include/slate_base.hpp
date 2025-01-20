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

#ifndef INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
#define INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

#include "base_driver.hpp"
#include "serial_driver.hpp"

enum class LightState : uint32_t
{
  LIGHT_STATE_OFF = 0,
  LIGHT_STATE_RED = 1,
  LIGHT_STATE_GREEN = 2,
  LIGHT_STATE_YELLOW = 3,
  LIGHT_STATE_BLUE = 4,
  LIGHT_STATE_PURPLE = 5,
  LIGHT_STATE_CYAN = 6,
  LIGHT_STATE_WHITE = 7,
  LIGHT_STATE_RED_FLASH = 9,
  LIGHT_STATE_GREEN_FLASH = 10,
  LIGHT_STATE_YELLOW_FLASH = 11,
  LIGHT_STATE_BLUE_FLASH = 12,
  LIGHT_STATE_PURPLE_FLASH = 13,
  LIGHT_STATE_CYAN_FLASH = 14,
  LIGHT_STATE_WHITE_FLASH = 15
};

namespace slate_base
{

#define CMD_TIME_OUT 300 // ms
#define PORT "chassis"

  class SlateBase
  {
  public:
    /**
     * @brief Constructor for the SlateBase
     */
    SlateBase();

    /// @brief Destructor for the SlateBase
    ~SlateBase() {}

    /// @brief Process velocity commands and update robot state
    void update();

    /// @brief Set velocity commands
    bool set_cmd_vel(float vel_x, float vel_z);

    /// @brief Set light state
    bool set_light_state(LightState light_state);

    /// @brief Set text
    bool set_text(std::string text);

    /// @brief Enable/disable motor torque
    bool enable_motor_torque(bool enable);

    /// @brief Enable/disable charging
    bool enable_charging(bool enable);

    /// @brief Get velocity commands
    std::vector<float> get_cmd_vel();

    /// @brief Get current pose
    std::vector<float> get_pose();

    /// @brief Get initial pose
    std::vector<float> get_init_pose();

  private:
    // Linear velocity command
    float cmd_vel_x_;

    // Angular velocity command
    float cmd_vel_z_;

    // Time last velocity command was received
    std::chrono::high_resolution_clock::time_point cmd_vel_time_last_update_;

    // Update counter used to only update some values less frequently
    int cnt_;

    // Odometry translation in the x-direction in meters
    float x_;

    // Odometry translation in the y-direction in meters
    float y_;

    // Odometry rotation about the z-axis in radians
    float theta_;

    // Odometry forward velocity in meters per second
    float x_vel_;

    // Odometry rotational velocity about the z-axis in radians per second
    float z_omega_;

    // Whether or not we have received our first odometry update
    bool is_first_odom_;

    // Array containing x and y translation in meters and rotation in radians
    float pose_[3];

    // Current of the right motor in Amps
    float right_motor_c_;

    // Current of the left motor in Amps
    float left_motor_c_;

    // The system state of the base
    SystemState chassis_state_;

    // Max linear velocity in the x-direction in meters per second
    float max_vel_x_ = 1.0;

    // Max angular velocity about the z-axis in radians per second
    float max_vel_z_ = 1.0;

    // Base command bytes containing data about charging and motor torque enabling
    uint32_t sys_cmd_ = 0;

    // Base light state - see interbotix_slate_msgs/srv/SetLightState for details
    uint32_t light_state_ = 0;

    // Time of the current update
    std::chrono::high_resolution_clock::time_point current_time_;

    // Timeout for base velocity
    std::chrono::duration<double> cmd_vel_timeout_;
  };

} // namespace slate_base

#endif // INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

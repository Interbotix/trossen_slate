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

#include "base_driver.hpp"
#include "serial_driver.hpp"

enum class LightState : uint8_t
{
  OFF = 0,
  RED,
  GREEN,
  YELLOW,
  BLUE,
  PURPLE,
  CYAN,
  WHITE,

  RED_FLASH = 9,
  GREEN_FLASH,
  YELLOW_FLASH,
  BLUE_FLASH,
  PURPLE_FLASH,
  CYAN_FLASH,
  WHITE_FLASH
};

namespace slate_base
{

#define CMD_TIME_OUT 300 // ms
#define PORT "chassis"

class SlateBase
{
public:
  /**
   * @brief Constructor for SlateBase
   */
  SlateBase();

  /// @brief Destructor for SlateBase
  ~SlateBase() {}

  /**
   * @brief Read data from the Slate base
   * @param data The desired data reference to update with current data
   */
  void read(base_driver::ChassisData & data);

  /**
   * @brief Write data to the Slate base
   * @param data The desired data to write
   */
  void write(base_driver::ChassisData data);

  /**
   * @brief Initializes Slate base
   * @param result The resulting output string
   */
  void init_base(std::string & result);

  /**
   * @brief Set velocity commands
   * @param linear The desired linear velocity in meters per second
   * @param angular The desired angular velocity in meters per second
   * @return true if succeeded, false otherwise
   */
  bool set_cmd_vel(float linear, float angular);

  /**
   * @brief Set light state
   * @param light_state The desired light state
   * @param result The resulting output string
   * @return true if succeeded, false otherwise
   */
  bool set_light_state(LightState light_state);

  /**
   * @brief Set text on screen
   * @param text The desired text
   * @param result The resulting output string
   * @return true if succeeded, false otherwise
   */
  bool set_text(std::string text);

  /**
   * @brief Enable/disable motor torque
   * @param enable Whether to enable motor torque or not
   * @param result The resulting output string
   * @return true if succeeded, false otherwise
   */
  bool enable_motor_torque(bool enable, std::string & result);

  /**
   * @brief Enable/disable charging
   * @param enable Whether to enable charging or not
   * @param result The resulting output string
   * @return true if succeeded, false otherwise
   */
  bool enable_charging(bool enable, std::string & result);

  /**
   * @brief Get velocity
   * @return The current velocity [linear velocity, angular velocity]
   */
  std::array<float, 2> get_vel();

  /**
   * @brief Get current pose
   * @return The current pose [x, y, theta]
   */
  std::array<float, 3> get_pose();

  /**
   * @brief Gets the current charge %
   * @return The current charge
   */
  uint32_t get_charge();

  /**
   * @brief Gets the current motor current in amps
   * @return The current motor current
   */
  float get_current();

  /**
   * @brief Gets the current voltage in volts
   * @return The current voltage
   */
  float get_voltage();

private:
  // Max linear velocity in the x-direction in meters per second
  float max_vel_x_ = 1.0;

  // Max angular velocity about the z-axis in radians per second
  float max_vel_z_ = 1.0;

  // Stored data of the Slate base - see base_driver.hpp for details
  base_driver::ChassisData data_;

  // Base command bytes containing data about charging and motor torque enabling
  uint32_t sys_cmd_ = 0;
};

} // namespace slate_base

#endif // INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

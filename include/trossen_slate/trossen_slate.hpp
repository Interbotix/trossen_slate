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

#ifndef TROSSEN_SLATE__TROSSEN_SLATE_HPP_
#define TROSSEN_SLATE__TROSSEN_SLATE_HPP_

#include <array>
#include <memory>
#include <string>

#include "trossen_slate/base_driver.hpp"
#include "trossen_slate/serial_driver.hpp"
#include "trossen_slate/version.hpp"

enum class LightState : uint32_t
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

namespace trossen_slate
{

#define CMD_TIME_OUT 300 // ms
#define PORT "chassis"

#define MAX_VEL_X 1.0f
#define MAX_VEL_Z 1.0f

class TrossenSlate
{
public:
  /**
   * @brief Constructor for TrossenSlate
   */
  TrossenSlate();

  /// @brief Destructor for TrossenSlate
  ~TrossenSlate() {}

  /**
   * @brief Read data from the SLATE base
   * @param data The desired data reference to update with current data
   */
  void read(base_driver::ChassisData & data);

  /**
   * @brief Write data to the SLATE base
   * @param data The desired data to write
   * @return true if succeeded, false otherwise
   */
  bool write(base_driver::ChassisData data);

  /**
   * @brief Update the state of the SLATE base
   * @return true if succeeded, false otherwise
   */
  bool update_state();

  /**
   * @brief Initializes the SLATE base
   * @param result The resulting output string
   * @return true if succeeded, false otherwise
   */
  bool init_base(std::string & result);

  /**
   * @brief Set velocity commands in meters per seconds (linear) and radians per seconds (angular)
   * @param linear The desired linear velocity
   * @param angular The desired angular velocity
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
   * @brief Get the current velocity in meters per seconds (linear) and radians per seconds (angular)
   * @return The current velocity [linear velocity, angular velocity]
   */
  std::array<float, 2> get_vel();

  /**
   * @brief Get the current pose in meters (x,y) and radians (theta)
   * @return The current pose [x, y, theta]
   */
  std::array<float, 3> get_pose();

  /**
   * @brief Get the current charge percentage
   * @return The current charge
   */
  uint32_t get_charge();

  /**
   * @brief Get the current motor current in amps
   * @return The current motor current
   */
  float get_current();

  /**
   * @brief Get the current voltage in volts
   * @return The current voltage
   */
  float get_voltage();

private:
  // Flag to keep track of base initialization
  bool base_initialized_ = false;

  // Stored data of the SLATE base - see base_driver.hpp for details
  base_driver::ChassisData data_;

  // Base command bytes containing data about charging and motor torque enabling
  uint32_t sys_cmd_ = 0;
};

} // namespace trossen_slate

#endif // TROSSEN_SLATE__TROSSEN_SLATE_HPP_

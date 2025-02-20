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

#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <string>

#include "trossen_slate/trossen_slate.hpp"
#include "trossen_slate/base_driver.hpp"

namespace py = pybind11;

namespace trossen_slate
{

PYBIND11_MODULE(trossen_slate, m)
{
  m.doc() = "Trossen SLATE Python Bindings";

  py::enum_<LightState>(m, "LightState")
  .value("OFF", LightState::OFF)
  .value("RED", LightState::RED)
  .value("GREEN", LightState::GREEN)
  .value("YELLOW", LightState::YELLOW)
  .value("BLUE", LightState::BLUE)
  .value("PURPLE", LightState::PURPLE)
  .value("CYAN", LightState::CYAN)
  .value("WHITE", LightState::WHITE)
  .value("RED_FLASH", LightState::RED_FLASH)
  .value("GREEN_FLASH", LightState::GREEN_FLASH)
  .value("YELLOW_FLASH", LightState::YELLOW_FLASH)
  .value("BLUE_FLASH", LightState::BLUE_FLASH)
  .value("PURPLE_FLASH", LightState::PURPLE_FLASH)
  .value("CYAN_FLASH", LightState::CYAN_FLASH)
  .value("WHITE_FLASH", LightState::WHITE_FLASH)
  .export_values();

  py::class_<base_driver::ChassisData>(m, "ChassisData")
  .def(py::init<>())
  .def_readwrite("cmd_vel_x", &base_driver::ChassisData::cmd_vel_x)
  .def_readwrite("cmd_vel_y", &base_driver::ChassisData::cmd_vel_y)
  .def_readwrite("cmd_vel_z", &base_driver::ChassisData::cmd_vel_z)
  .def_readwrite("light_state", &base_driver::ChassisData::light_state)
  .def_readwrite("system_state", &base_driver::ChassisData::system_state)
  .def_readwrite("charge", &base_driver::ChassisData::charge)
  .def_readwrite("voltage", &base_driver::ChassisData::voltage)
  .def_readwrite("current", &base_driver::ChassisData::current)
  .def_readwrite("vel_x", &base_driver::ChassisData::vel_x)
  .def_readwrite("vel_y", &base_driver::ChassisData::vel_y)
  .def_readwrite("vel_z", &base_driver::ChassisData::vel_z)
  .def_readwrite("odom_x", &base_driver::ChassisData::odom_x)
  .def_readwrite("odom_y", &base_driver::ChassisData::odom_y)
  .def_readwrite("odom_z", &base_driver::ChassisData::odom_z)
  .def_readwrite("cmd", &base_driver::ChassisData::cmd)
  .def_readwrite("io", &base_driver::ChassisData::io)
  .def_readwrite("err", &base_driver::ChassisData::err);

  py::class_<TrossenSlate>(m, "TrossenSlate")
  .def(py::init<>())
  .def(
    "read",
    &TrossenSlate::read,
    py::arg(
      "data"),
    R"pbdoc(
      @brief Read data from the SLATE base
      
      @param data The desired data reference to update with current data
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "write",
    &TrossenSlate::write,
    py::arg(
      "data"),
    R"pbdoc(
      @brief Write data to the SLATE base
      
      @param data The desired data to write
      @return true if succeeded, false otherwise
    )pbdoc")
    .def(
      "update_state",
    &TrossenSlate::update_state,
    R"pbdoc(
      @brief Update the state of the SLATE base
      
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "init_base",
    [](TrossenSlate & self)
    {
      std::string result;
      bool success = self.init_base(result);
      return std::make_pair(
        success,
        result);
    },
    R"pbdoc(
      @brief Initializes the SLATE base
      
      @param result The resulting output string
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "set_cmd_vel",
    &TrossenSlate::set_cmd_vel,
    py::arg("linear"),
    py::arg(
      "angular"),
    R"pbdoc(
      @brief Set velocity commands in meters per seconds (linear) and radians per seconds (angular)
      
      @param linear The desired linear velocity
      @param angular The desired angular velocity
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "set_light_state",
    &TrossenSlate::set_light_state,
    py::arg(
      "light_state"),
    R"pbdoc(
      @brief Set light state
      
      @param light_state The desired light state
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "set_text",
    &TrossenSlate::set_text,
    py::arg(
      "text"),
    R"pbdoc(
      @brief Set text on screen
      
      @param text The desired text
      @return true if succeeded, false otherwise
    )pbdoc")
  .def(
    "enable_motor_torque",
    [](TrossenSlate & self, bool enable)
    {
      std::string result;
      bool success = self.enable_motor_torque(enable, result);
      return std::make_pair(success, result);
    },
    py::arg(
      "enable"),
    R"pbdoc(
      @brief Enable/disable motor torque
      
      @param enable Whether to enable motor torque or not
      @return A pair (bool, string) indicating success and the resulting output string
    )pbdoc"
  )
  .def(
    "enable_charging",
    [](TrossenSlate & self, bool enable)
    {
      std::string result;
      bool success = self.enable_charging(enable, result);
      return std::make_pair(success, result);
    },
    py::arg(
      "enable"),
    R"pbdoc(
      @brief Enable/disable charging
      
      @param enable Whether to enable charging or not
      @return A pair (bool, string) indicating success and the resulting output string
    )pbdoc"
  )
  .def(
    "get_vel",
    &TrossenSlate::get_vel,
    R"pbdoc(
      @brief Get the current velocity in meters per seconds (linear) and radians per seconds (angular)
      
      @return The current velocity [linear velocity, angular velocity]
    )pbdoc")
  .def(
    "get_pose",
    &TrossenSlate::get_pose,
    R"pbdoc(
      @brief Get the current pose in meters (x,y) and radians (theta)
      
      @return The current pose [x, y, theta]
    )pbdoc")
  .def(
    "get_charge",
    &TrossenSlate::get_charge,
    R"pbdoc(
      @brief Get the current charge percentage
      
      @return The current charge
    )pbdoc")
  .def(
    "get_current",
    &TrossenSlate::get_current,
    R"pbdoc(
      @brief Get the current motor current in amps
      
      @return The current motor current
    )pbdoc")
  .def(
    "get_voltage",
    &TrossenSlate::get_voltage,
    R"pbdoc(
      @brief Get the current voltage in volts
      
      @return The current voltage
    )pbdoc");
}
} // namespace trossen_slate

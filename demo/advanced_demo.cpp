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

// This demo showcases how to use more detailed methods to control and monitor the SLATE base.

#include <iostream>

#include "trossen_slate/trossen_slate.hpp"

using namespace trossen_slate;

int main()
{
  // Create TrossenSlate object
  TrossenSlate slate;

  // Initialize base and output result
  std::string result_init;
  slate.init_base(result_init);
  std::cout << result_init << std::endl;

  // Disable charging and output result
  std::string result_charging;
  slate.enable_charging(false, result_charging);
  std::cout << result_charging << std::endl;

  // Enable motor torque and output result
  std::string result_torque;
  slate.enable_motor_torque(true, result_torque);
  std::cout << result_torque << std::endl;

  while (true) {
    // Initialize data with angular velocity and light state
    base_driver::ChassisData my_data = {
      .cmd_vel_z = -0.1,
      .light_state = static_cast<uint32_t>(LightState::WHITE)};

    // Write and update base data
    slate.write(my_data);

    // Initialize empty log data
    base_driver::ChassisData log_data;

    // Read and output data
    slate.read(log_data);

    std::cout << "Charge: " << log_data.charge << "%" << std::endl;
    std::cout << "Linear velocity: " << log_data.vel_x << " Angular velocity: " << log_data.vel_z <<
      std::endl;
    std::cout << "X: " << log_data.odom_x << " Y: " << log_data.odom_y << " Theta: " <<
      log_data.odom_z << std::endl;
  }

  return 0;
}

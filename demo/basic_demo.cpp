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

// This demo showcases how to use basic methods to control and the SLATE base.

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

  // Display "Hello world" on screen
  slate.set_text("Hello world");

  while (true) {
    // Set the LED colors to PURPLE
    slate.set_light_state(LightState::PURPLE);

    // Set angular velocity to 0.1
    slate.set_cmd_vel(0.0, 0.1);

    // Output charge percentage
    std::cout << "Charge: " << slate.get_charge() << "%" << std::endl;

    // Get velocity and output values
    std::array<float, 2> vel = slate.get_vel();
    std::cout << "Linear velocity: " << vel[0] << " Angular velocity: " << vel[1] << std::endl;

    // Get pose and output values
    std::array<float, 3> pose = slate.get_pose();
    std::cout << "X: " << pose[0] << " Y: " << pose[1] << " Theta: " << pose[2] << std::endl;
  }

  return 0;
}

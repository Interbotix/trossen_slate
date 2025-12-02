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

// This demo showcases how to reset the odometry of the SLATE base.

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "trossen_slate/trossen_slate.hpp"


trossen_slate::TrossenSlate* slate_ptr = nullptr;

void sigint_handler(int signum)
{
  if (slate_ptr != nullptr) {
    slate_ptr->set_cmd_vel(0.0, 0.0);
    std::cout << "\nInterrupt occurred. Stopping robot..." << std::endl;
  }
  exit(signum);
}

int main()
{
  // Create TrossenSlate object
  trossen_slate::TrossenSlate slate;
  slate_ptr = &slate;

  // Register signal handler
  std::signal(SIGINT, sigint_handler);

  // Initialize base and output result
  std::string result_init;
  slate.init_base(result_init);
  std::cout << result_init << std::endl;

  // Disable motor torque
  std::string result_torque;
  slate.enable_motor_torque(false, result_torque);
  std::cout << result_torque << std::endl;

  // Print odometry before reset
  std::cout << "\n=== Odometry Before Reset ===" << std::endl;
  for (int i = 0; i < 5; i++) {
    std::array<float, 3> pose = slate.get_pose();
    std::cout
      << "X: " << pose[0]
      << " m, Y: " << pose[1]
      << " m, Theta: " << pose[2]
      << " rad" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Wait for user to press Enter
  std::cout << "\nMove the base around if you'd like, then press Enter to reset odometry..." << std::endl;
  std::cin.get();

  // Print off odom before reset
  std::array<float, 3> pose_before = slate.get_pose();
  std::cout
    << "Odometry before reset - X: " << pose_before[0]
    << " m, Y: " << pose_before[1]
    << " m, Theta: " << pose_before[2]
    << " rad" << std::endl;

  // Reset odometry
  std::string result_reset;
  slate.reset_odometry(result_reset);
  std::cout << result_reset << std::endl;

  // Continue printing odometry after reset
  std::cout << "\n=== Odometry After Reset ===" << std::endl;
  for (int i = 0; i < 10; i++) {
    std::array<float, 3> pose = slate.get_pose();
    std::cout
      << "X: " << pose[0]
      << " m, Y: " << pose[1]
      << " m, Theta: " << pose[2]
      << " rad" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "\nDemo complete." << std::endl;

  return 0;
}

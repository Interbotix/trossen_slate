#include "slate_base.hpp"
#include <iostream>
using namespace slate_base;

int main()
{
  // Create SlateBase object
  SlateBase slate;

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

    // Output current charge percentage
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

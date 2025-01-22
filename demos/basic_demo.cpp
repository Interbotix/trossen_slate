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

    // Get velocity and output value
    std::array<float, 2> vel = slate.get_vel();
    std::cout << vel[1] << std::endl;
  }

  return 0;
}

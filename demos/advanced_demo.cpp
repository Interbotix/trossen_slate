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

  //  Disable charging and output result
  std::string result_charging;
  slate.enable_charging(false, result_charging);
  std::cout << result_charging << std::endl;

  //  Enable motor torque and output result
  std::string result_torque;
  slate.enable_motor_torque(true, result_torque);
  std::cout << result_torque << std::endl;

  while (true) {
    // Initialize data with angular velocity and light state (WHITE)
    base_driver::ChassisData my_data = {
      .cmd_vel_z = -0.1,
      .light_state = 7};

    // Write and update base data
    slate.write(my_data);

    // Initialize empty log data
    base_driver::ChassisData log_data;

    // Read and output data
    slate.read(log_data);
    std::cout << log_data.vel_z << std::endl;
    std::cout << log_data.charge << std::endl;
  }

  return 0;
}

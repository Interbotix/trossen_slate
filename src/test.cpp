#include "slate_base.hpp"
#include <iostream>
using namespace slate_base;

int main()
{
  SlateBase slate;
  slate.set_light_state(LightState::LIGHT_STATE_GREEN_FLASH);
  slate.set_text("Hello world");
  slate.enable_charging(true);
  slate.enable_motor_torque(true);
  slate.set_cmd_vel(-0.2, 0.0);

  while (true)
  {
    slate.update();
    std::vector<float> pose = slate.get_pose();
    std::vector<float> init_pose = slate.get_init_pose();
    std::vector<float> vel = slate.get_cmd_vel();

    std::cout << "Initial pose:" << " x: " << init_pose[0] << " y: " << init_pose[1] << " theta: " << init_pose[2] << std::endl;
    std::cout << "Current pose: " << "x: " << pose[0] << " y: " << pose[1] << " theta: " << pose[2] << std::endl;
    std::cout << "Current velocity: " << "linear: " << vel[0] << " angular: " << vel[1] << std::endl;
  }

  return 0;
}
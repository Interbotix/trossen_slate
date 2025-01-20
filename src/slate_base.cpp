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

#include "slate_base.hpp"

namespace slate_base
{
    SlateBase::SlateBase() : cmd_vel_x_(0.0), cmd_vel_z_(0.0), cnt_(0), x_(0.0),
                             y_(0.0), theta_(0.0), x_vel_(0.0), z_omega_(0.0), is_first_odom_(true), pose_{0},
                             right_motor_c_(0.0), left_motor_c_(0.0), chassis_state_(SystemState::SYS_INIT),
                             max_vel_x_(1.0), max_vel_z_(1.0), current_time_(std::chrono::high_resolution_clock::now()), cmd_vel_time_last_update_(std::chrono::high_resolution_clock::now()), cmd_vel_timeout_(std::chrono::milliseconds(CMD_TIME_OUT))
    {
        std::string dev;
        if (!base_driver::chassisInit(dev))
        {
            std::cout << "Failed to initialize base port." << std::endl;
            return;
        }
        std::cout << "Initalized base at port " << dev.c_str() << std::endl;
        char version[32] = {0};
        if (base_driver::getVersion(version))
        {
            std::cout << "Base version: " << version << std::endl;
        }
    }

    void SlateBase::update()
    {
        current_time_ = std::chrono::high_resolution_clock::now();

        // time out velocity commands
        if (current_time_ - cmd_vel_time_last_update_ > cmd_vel_timeout_)
        {
            cmd_vel_x_ = 0.0f;
            cmd_vel_z_ = 0.0f;
        }

        // limit velocity commands
        cmd_vel_x_ = std::min(max_vel_x_, std::max(-max_vel_x_, cmd_vel_x_));
        cmd_vel_z_ = std::min(max_vel_z_, std::max(-max_vel_z_, cmd_vel_z_));

        // initialize chassis data and use it to update the driver
        base_driver::ChassisData data = {
            .cmd_vel_x = cmd_vel_x_,
            .cmd_vel_y = 0.0,
            .cmd_vel_z = cmd_vel_z_,
            .light_state = light_state_,
            .system_state = 0};

        if (!base_driver::updateChassisInfo(&data))
        {
            return;
        }

        // extract and update base system command bytes
        sys_cmd_ = data.cmd;

        // update odometry values
        x_vel_ = data.vel_x;
        z_omega_ = data.vel_z;

        x_ = data.odom_x;
        y_ = data.odom_y;
        theta_ = data.odom_z;

        if (is_first_odom_)
        {
            pose_[0] = x_;
            pose_[1] = y_;
            pose_[2] = theta_;
            is_first_odom_ = false;
        }
    }

    bool SlateBase::set_cmd_vel(float vel_x, float vel_z)
    {
        cmd_vel_x_ = vel_x;
        cmd_vel_z_ = vel_z;
        cmd_vel_time_last_update_ = std::chrono::high_resolution_clock::now();
        return true;
    }

    bool SlateBase::set_light_state(LightState light_state)
    {
        std::cout << "Set light state to: '" << static_cast<uint32_t>(light_state) << "'." << std::endl;
        light_state_ = static_cast<uint8_t>(light_state);
        return true;
    }

    bool SlateBase::set_text(std::string text)
    {
        std::cout << "Set base screen text to: '" << text.c_str() << "'." << std::endl;
        base_driver::setText(text.c_str());
        return true;
    }

    bool SlateBase::enable_motor_torque(bool enable)
    {
        enable ? sys_cmd_ &= ~(1) : sys_cmd_ |= 1;
        bool success = base_driver::setSysCmd(sys_cmd_);
        std::string enabled_disabled = enable ? "enable" : "disable";

        if (success)
        {
            std::cout << "Successfully " << enabled_disabled << "d motor torque." << std::endl;
        }
        else
        {
            std::cout << "Failed to " << enabled_disabled << " motor torque." << std::endl;
        }
        return success;
    }

    bool SlateBase::enable_charging(bool enable)
    {
        enable ? sys_cmd_ &= ~(2) : sys_cmd_ |= 2;
        bool success = base_driver::setSysCmd(sys_cmd_);

        std::string enabled_disabled = enable ? "enable" : "disable";

        if (success)
        {
            std::cout << "Successfully " << enabled_disabled << "d charging." << std::endl;
        }
        else
        {
            std::cout << "Failed to " << enabled_disabled << " charging." << std::endl;
        }
        return success;
    }

    std::vector<float> SlateBase::get_cmd_vel()
    {
        std::vector<float> cmd_vel(2);
        cmd_vel[0] = x_vel_;
        cmd_vel[1] = z_omega_;
        return cmd_vel;
    }

    std::vector<float> SlateBase::get_pose()
    {
        std::vector<float> pose(2);
        pose[0] = x_;
        pose[1] = y_;
        pose[2] = theta_;
        return pose;
    }

    std::vector<float> SlateBase::get_init_pose()
    {
        std::vector<float> pose(2);
        pose[0] = pose_[0];
        pose[1] = pose_[1];
        pose[2] = pose_[2];
        return pose;
    }

} // namespace slate_base
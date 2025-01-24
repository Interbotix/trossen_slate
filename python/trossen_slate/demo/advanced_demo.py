# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This demo showcases how to use more detailed methods to control and monitor the SLATE base."""

import trossen_slate as trossen

def main():
    # Create TrossenSlate object
    slate = trossen.TrossenSlate()

    # Initialize base and output result
    success, result_init = slate.init_base()
    print(f"Initialization success: {success}\nResult: {result_init}")

    # Disable charging and output result
    success, result_charging = slate.enable_charging(False)
    print(f"Disable charging success: {success}\nResult: {result_charging}")

    # Enable motor torque and output result
    success, result_torque = slate.enable_motor_torque(True)
    print(f"Enable motor torque success: {success}\nResult: {result_torque}")

    while True:
        # Initialize data with angular velocity and light state
        my_data = trossen.ChassisData()
        my_data.cmd_vel_z = -0.1
        my_data.light_state = trossen.LightState.WHITE

        # Write and update base data
        slate.write(my_data)

        # Initialize empty log data
        log_data = trossen.ChassisData()

        # Read and output data
        slate.read(log_data)

        print(f"Charge: {log_data.charge}%")
        print(f"Linear velocity: {log_data.vel_x:.2f} Angular velocity: {log_data.vel_z:.2f}")
        print(f"X: {log_data.odom_x:.2f} Y: {log_data.odom_y:.2f} Theta: {log_data.odom_z:.2f}")

if __name__ == "__main__":
    main()

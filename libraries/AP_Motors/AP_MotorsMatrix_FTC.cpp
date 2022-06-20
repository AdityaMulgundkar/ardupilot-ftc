/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// This allows motor roll, pitch, yaw and throttle factors to be changed in flight, allowing vehicle geometry to be changed

#include "AP_MotorsMatrix_FTC.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// add a motor and give its testing order
bool AP_MotorsMatrix_FTC::add_motor(uint8_t motor_num, uint8_t testing_order)
{
    if (initialised_ok()) {
        // no adding motors after init
        return false;
    }
    if (motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        _test_order[motor_num] = testing_order;
        motor_enabled[motor_num] = true;
        return true;
    }
    return false;
}

void AP_MotorsMatrix_FTC::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Make sure the correct number of motors have been added
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    _mav_type = MAV_TYPE_HEXAROTOR;
    set_update_rate(_speed_hz);
}

// output - sends commands to the motors, 
// Need to take the semaphore to enasure the motor factors are not changed during the mixer calculation
void AP_MotorsMatrix_FTC::output_to_motors()
{
    // call the base class ouput function
    AP_MotorsMatrix::output_to_motors();
}

// singleton instance
AP_MotorsMatrix_FTC *AP_MotorsMatrix_FTC::_singleton;
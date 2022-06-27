#include "Copter.h"
#include <math.h>

#define ALT_HOLD_MS 10000 // Stay at dest_alt for this much ms
#define M1_FAULT_MS 100 // Time since M1 fault in ms

uint32_t _time_since_init = 4294967295; 
uint32_t _time_since_alt_reached = 4294967295; 
uint32_t _time_since_m1_fault = 4294967295; 

bool alt_flag = false;
bool elapsed_flag = false;
bool m1_fault = false;
bool fault_fixed = false;

float rolls[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};
float pitches[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};

AP_MotorsMatrix_FTC *motor_object;

/*
 * Init and run calls for FTC flight mode
 */

// ftc_1_init - initialise FTC controller
bool ModeFTC1::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 init()");
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    motor_object = new AP_MotorsMatrix_FTC(copter.scheduler.get_loop_rate_hz());
    // check board has initialised
    if (!copter.position_ok()) {
        return false;
    }
    _time_since_init = AP_HAL::millis();
    return true;
}

// ftc_1_run - runs the main FTC controller
// should be called at 100hz or more
void ModeFTC1::run()
{
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run()");
    // Disable selected motor
    motor_object->remove_motor(0);
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() remove motor 0");
    // Reconfigure motor effectiveness (rpy factors)
    /* Calculations:
    // RAW: (Incorrect numbering) (Correct no. on right suffix)
    add_motor_raw(AP_MOTORS_MOT_1, -0.808, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);  - 1st
    add_motor_raw(AP_MOTORS_MOT_2, 0.346, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);    - 6th
    add_motor_raw(AP_MOTORS_MOT_3, 0.924, 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);   - 5th
    add_motor_raw(AP_MOTORS_MOT_4, 0.346, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);   - 4th
    add_motor_raw(AP_MOTORS_MOT_5, -0.808, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5); - 3rd
    // SRC 2-5-1-3-6-4
    add_motor_raw(AP_MOTORS_MOT_2, 0.924, 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
    add_motor_raw(AP_MOTORS_MOT_3, -0.808, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
    add_motor_raw(AP_MOTORS_MOT_4, -0.808, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
    add_motor_raw(AP_MOTORS_MOT_5, 0.346, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6);
    add_motor_raw(AP_MOTORS_MOT_6, 0.346, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4);
    */
    motor_object->set_roll_factor(rolls);
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() rolls");
   
    motor_object->set_pitch_factor(pitches);
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() pitches");
    // Exit mode? Or stabilize?
    // normalise_rpy_factors
    // motor_object->normalise_rpy_factors();
}
#include "Copter.h"
#include <math.h>

#define ALT_HOLD_MS 10000 // Stay at dest_alt for this much ms
#define M1_FAULT_MS 100 // Time since M1 fault in ms

uint32_t _time_since_init = 4294967295; 
uint32_t _time_since_alt_reached = 4294967295; 
uint32_t _time_since_m1_fault = 4294967295; 

bool vars_flag = false;
bool takeoff_complete = false;

// float rolls[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};
// float pitches[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};
float target_climb_rate = 1000;

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
    // motor_object = motor_object->get_singleton();
    // motor_object = new AP_MotorsMatrix_FTC(copter.scheduler.get_loop_rate_hz());
    // check board has initialised
    if (!copter.position_ok()) {
        return false;
    }

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    _time_since_init = AP_HAL::millis();
    return true;
}

// ftc_1_run - runs the main FTC controller
// should be called at 100hz or more
void ModeFTC1::run()
{
    gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run()");
    if(vars_flag) {
        // gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() Frame: %s", motor_object->_get_frame_string());
        gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() remove motor 0");
        // Disable selected motor
        // motor_object->remove_motor(0);

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
        // motor_object->set_roll_factor(rolls);
        gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() rolls");
    
        // motor_object->set_pitch_factor(pitches);
        gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() pitches");
        // Exit mode? Or stabilize?
        // normalise_rpy_factors
        // motor_object->normalise_rpy_factors();
        vars_flag = true;
    }
    else {
        gcs().send_text(MAV_SEVERITY_INFO, "ModeFTC1 run() climbing");
        // pos_control->set_pos_target_z_cm(1000);
        // Send the commanded climb rate to the position controller
        // pos_control->set_pos_target_z_from_climb_rate_cm(1000);

        // pos_control->update_z_controller();
        

        // calculate current and target altitudes
        // by default current_alt_cm and alt_target_cm are alt-above-EKF-origin
        int32_t alt_target_cm = 1000;
        bool alt_target_terrain = false;
        float current_alt_cm = inertial_nav.get_position_z_up_cm();
        // float terrain_offset;   // terrain's altitude in cm above the ekf origin

        // sanity check target
        int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
        alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

        // initialise yaw
        auto_yaw.set_mode(AUTO_YAW_HOLD);

        // clear i term when we're taking off
        set_throttle_takeoff();

        // initialise alt for WP_NAVALT_MIN and set completion alt
        auto_takeoff_start(alt_target_cm, alt_target_terrain);

        // init wpnav and set origin if transitioning from takeoff
        if (!wp_nav->is_active()) {
            Vector3f stopping_point;
                Vector3p takeoff_complete_pos;
                if (auto_takeoff_get_position(takeoff_complete_pos)) {
                    stopping_point = takeoff_complete_pos.tofloat();
                }
            wp_nav->wp_and_spline_init(0, stopping_point);
        }
        // run the vertical position controller and set output throttle
        pos_control->update_z_controller();
    }
}
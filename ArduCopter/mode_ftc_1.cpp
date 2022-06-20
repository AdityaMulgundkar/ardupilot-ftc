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

/*
 * Init and run calls for FTC flight mode
 */

// ftc_1_init - initialise FTC controller
bool ModeFTC1::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }

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
    // Disable selected motor
    // motors->remove_motor(0);
    // copter.remove_motor(0);
    // Reconfigure motor effectiveness (rpy factors)

    // Exit mode? Or stabilize?

    // get pilot desired climb rate
    int32_t dest_alt = 5000;
    int32_t curr_alt = copter.current_loc.alt;
    
    if(elapsed_flag==false) {
        gcs().send_text(MAV_SEVERITY_INFO, "alt_flag==FALSE");
        int low_pwm = 1200;
        int mid_pwm = 1500;
        int high_pwm = 1800;
        int o_pwm = map_ranges(0, dest_alt, 300, 150, curr_alt);
        
        low_pwm = mid_pwm - o_pwm;
        high_pwm = mid_pwm + o_pwm;

        if(curr_alt > dest_alt) {
            motors->rc_write(AP_MOTORS_MOT_1, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_2, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_3, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_4, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_5, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_6, low_pwm);
        }
        else {
            motors->rc_write(AP_MOTORS_MOT_1, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_2, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_3, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_4, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_5, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_6, high_pwm);
        }
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    if(curr_alt > 0.95 * dest_alt && curr_alt < dest_alt) {
        if(alt_flag == false) {
            alt_flag = true;
            gcs().send_text(MAV_SEVERITY_INFO, "alt_flag==TRUE");
            _time_since_alt_reached = AP_HAL::millis();
        }
        if(AP_HAL::millis() - _time_since_alt_reached > ALT_HOLD_MS && alt_flag) {
            // 10 sec elapsed
            elapsed_flag = true;
            gcs().send_text(MAV_SEVERITY_INFO, "elapsed_flag==TRUE");
        }
    }

    if (elapsed_flag && !m1_fault) {
        // Time to bring a fault
        gcs().send_text(MAV_SEVERITY_INFO, "m1_fault==TRUE");
        m1_fault = true;
        motors->rc_write(AP_MOTORS_MOT_1, 1000);
        _time_since_m1_fault = AP_HAL::millis();
    }

    if (AP_HAL::millis() - _time_since_m1_fault > M1_FAULT_MS && m1_fault == true) {
        // Time to fix the fault after M1_FAULT_MS ms delay
        if(!fault_fixed) {
            gcs().send_text(MAV_SEVERITY_INFO, "fault_fixed==TRUE");
            fault_fixed = true;
            motors->rc_write(AP_MOTORS_MOT_2, 1000);
            motors->rc_write(AP_MOTORS_MOT_3, 1900);
            motors->rc_write(AP_MOTORS_MOT_4, 1900);
            motors->rc_write(AP_MOTORS_MOT_5, 1900);
            motors->rc_write(AP_MOTORS_MOT_6, 1900);
        }
    }

    if(fault_fixed) {
        gcs().send_text(MAV_SEVERITY_INFO, "fault_fixed entered");

        int low_pwm = 1200;
        int mid_pwm = 1500;
        int high_pwm = 1900;
        float curr_roll = AP::ahrs().get_roll();
        float curr_pitch = AP::ahrs().get_pitch();
        
        if(abs(curr_pitch) > 0.05) {
            // 56 vs 43; Motor setup;
            // left-high (56) means positive pitch
            low_pwm = 1700;
            high_pwm = 1800;

            if(curr_pitch < -0.05) {
                gcs().send_text(MAV_SEVERITY_INFO, "Pitch NEG: %f", curr_pitch);
                motors->rc_write(AP_MOTORS_MOT_3, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, low_pwm);
            }
            if(curr_pitch > 0.05) {
                gcs().send_text(MAV_SEVERITY_INFO, "Pitch POS: %f", curr_pitch);
                motors->rc_write(AP_MOTORS_MOT_3, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, high_pwm);
            }
        }
        else if(abs(curr_roll) > 0.1) {
            // 45 vs 36; Motor setup;
            // left-high (45) means positive roll
            low_pwm = 1700;
            high_pwm = 1800;

            if(curr_roll < -0.1) {
                gcs().send_text(MAV_SEVERITY_INFO, "Roll NEG: %f", curr_roll);
                motors->rc_write(AP_MOTORS_MOT_3, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, high_pwm);
            }
            if(curr_roll > 0.1) {
                gcs().send_text(MAV_SEVERITY_INFO, "Roll POS: %f", curr_roll);
                motors->rc_write(AP_MOTORS_MOT_3, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, low_pwm);
            }
        }
        else {
            gcs().send_text(MAV_SEVERITY_INFO, "Reaching alt");
            int l_pwm = map_ranges(0, dest_alt, 200, 100, curr_alt);
            int h_pwm = map_ranges(0, dest_alt, 400, 250, curr_alt);
            
            low_pwm = mid_pwm - l_pwm;
            high_pwm = mid_pwm + h_pwm;

            if(curr_alt > dest_alt) {
                motors->rc_write(AP_MOTORS_MOT_3, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, low_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, low_pwm);
            }
            else {
                motors->rc_write(AP_MOTORS_MOT_3, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_4, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_5, high_pwm);
                motors->rc_write(AP_MOTORS_MOT_6, high_pwm);
            }
        }
        /*
        int low_pwm = 1200;
        int mid_pwm = 1500;
        int high_pwm = 1900;

        int l_pwm = map_ranges(0, dest_alt, 200, 100, curr_alt);
        int h_pwm = map_ranges(0, dest_alt, 400, 100, curr_alt);
            
        low_pwm = mid_pwm - l_pwm;
        high_pwm = mid_pwm + h_pwm;

        if(curr_alt > dest_alt) {
            motors->rc_write(AP_MOTORS_MOT_3, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_4, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_5, low_pwm);
            motors->rc_write(AP_MOTORS_MOT_6, low_pwm);
        }
        else {
            motors->rc_write(AP_MOTORS_MOT_3, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_4, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_5, high_pwm);
            motors->rc_write(AP_MOTORS_MOT_6, high_pwm);
        }
        */
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}

double round(double d)
{
    return floor(d + 0.5);
}

int ModeFTC1::map_ranges(int input_start, int input_end, int output_start, int output_end, int input) {
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    return output_start + round(slope * (input - input_start));
}
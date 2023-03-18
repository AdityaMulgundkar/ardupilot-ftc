#include "Copter.h"

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz

AP_MotorsMatrix_FTC *motor_object;
AP_MotorsMulticopter *motors;

float rolls[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};
// float rolls[6] = {0.0, 0.0, -0.346, -0.346, 0.346, 0.346};
float pitches[6] = {0.0, 0.0, 1, -1, 1, -1};
float yaws[6] = {-1.0, 1.0, -1.0, 1.0, 1.0, -1.0};

float new_rolls[6] = {0, 0, 0, 0, 0 ,0};
float new_pitches[6] = {0, 0, 0, 0, 0 ,0};
float new_throttles[6] = {0, 0, 0, 0, 0 ,0};

int orders[6] = {2, 5, 6, 3, 1, 4};
float throttles[6] = {0.0, 1.0, 1.0, 1.0, 1.0, 1.0};

int old_flag = -1;
float m_factor = 1;

void Copter::ftc_listener_1()
{
    motor_object = motor_object->get_singleton();
    int ftc_flag_1 = g.ftc_flag_1;

    if(ftc_flag_1 != old_flag) {
        gcs().send_text(MAV_SEVERITY_INFO, "FTC FLAG: %d", ftc_flag_1);

        motor_object->set_initialised_ok(false);

        if(ftc_flag_1 != -1){
            motor_object->remove_motor(ftc_flag_1);
            motors->rc_write(0, 1000);
        }

        for(int k = 0; k < 6; k++) {
            new_rolls[k] = m_factor * rolls[k];
            // new_pitches[k] = m_factor * pitches[k];
            // new_throttles[k] = m_factor * throttles[k];
        }

        motor_object->set_roll_factor(new_rolls);
        motor_object->set_pitch_factor(pitches);
        motor_object->set_throttle_factor(throttles);

        gcs().send_text(MAV_SEVERITY_INFO, "FTC ROLLS AND PITCHES");

        motor_object->normalise_rpy_factors();
        gcs().send_text(MAV_SEVERITY_INFO, "FTC normalise_rpy_factors");

        motor_object->set_initialised_ok(true);
        gcs().send_text(MAV_SEVERITY_INFO, "FTC set_initialised_ok");

        motor_object->set_update_rate(copter.scheduler.get_loop_rate_hz());
        // motor_object->output();

        // calculate thrust
        // motor_object->output_armed_stabilizing();

        // convert rpy_thrust values to pwm
        // motor_object->output_to_motors();

        // output raw roll/pitch/yaw/thrust
        // motor_object->output_rpyt();

        old_flag = ftc_flag_1;
    }
}

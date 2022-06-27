#include "Copter.h"

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz

AP_MotorsMatrix_FTC *motor_object;

float rolls[6] = {0.0, 0.924, -0.808, -0.808, 0.346, 0.346};
float pitches[6] = {0.0, 0.0, 1, -1, 1, -1};
float yaws[6] = {-1.0, 1.0, -1.0, 1.0, 1.0, -1.0};
int orders[6] = {2, 5, 6, 3, 1, 4};

int old_flag = -1;

void Copter::ftc_listener_1()
{
    motor_object = motor_object->get_singleton();
    int ftc_flag_1 = g.ftc_flag_1;

    if(ftc_flag_1 != old_flag) {
        gcs().send_text(MAV_SEVERITY_INFO, "FTC FLAG: %d", ftc_flag_1);

        motor_object->set_initialised_ok(false);

        // remove existing motors
        for (int8_t j = 0; j < 6; j++) {
            motor_object->remove_motor(j);
            gcs().send_text(MAV_SEVERITY_INFO, "FTC REMOVING MOTOR: %d", j);
        }

        // motor_object->remove_motor(0);
        // gcs().send_text(MAV_SEVERITY_INFO, "FTC REMOVING MOTOR: %d", 0);
        
        // add back motors
        for (int8_t i = 0; i < 6; i++) {
            if(ftc_flag_1 != i) {
                motor_object->re_enable_motor(i);
                gcs().send_text(MAV_SEVERITY_INFO, "FTC RE-ENABLING MOTOR: %d", i);
            }
        }

        motor_object->set_roll_factor(rolls);
        motor_object->set_pitch_factor(pitches);

        gcs().send_text(MAV_SEVERITY_INFO, "FTC ROLLS AND PITCHES");

        /*
        // add new existing motors
        for (int8_t i = 0; i < 6; i++) {
            gcs().send_text(MAV_SEVERITY_INFO, "FTC ADDING MOTOR: %d", i);
            motor_object->add_motor_raw(
            i,
            rolls[i],
            pitches[i],
            yaws[i],
            orders[i]);
        }
        */

        /*
        static const AP_MotorsMatrix_FTC::MotorDefRaw motors_new[] {
            { 0.0, 0.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            { 0.924, 0.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -0.808, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            { -0.808, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            { 0.346, 1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { 0.346, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        };
        motor_object->add_motors_raw(motors_new, ARRAY_SIZE(motors_new));
        */

        // Disable selected motor
        // motor_object->remove_motor(ftc_flag_1);

        // motor_object->set_roll_factor(rolls);
        // motor_object->set_pitch_factor(pitches);

        motor_object->normalise_rpy_factors();

        motor_object->set_initialised_ok(true);

        // motor_object->output_armed_stabilizing();

        old_flag = ftc_flag_1;
    }
}

#include "Copter.h"

uint16_t last_control_outputs[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

/*
 * Init and run calls for FTC flight mode
 */

// ftc_1_init - initialise FTC controller
bool ModeFTC1::init(bool ignore_checks)
{
    return true;
}

// stop mission when we leave auto mode
void ModeFTC1::exit()
{
    // // if (copter.mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
    // //     copter.mode_auto.mission.stop();
    // // }
    // // initialise the vertical position controller
    // if (!pos_control->is_active_z()) {
    //     pos_control->init_z_controller();
    // }

    // // set vertical speed and acceleration limits
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    gcs().send_text(MAV_SEVERITY_INFO, "Exiting MODE FTC_1");
}

void ModeFTC1::parse_outputs(uint16_t *controls)
{
    last_control_outputs[0] = controls[0];
    last_control_outputs[1] = controls[1];
    last_control_outputs[2] = controls[2];
    last_control_outputs[3] = controls[3];
}

// ftc_1_run - runs the main FTC controller
// should be called at 100hz or more
void ModeFTC1::run()
{
    if (copter.motors->armed())
    {
        copter.motors->rc_write(AP_MOTORS_MOT_1, last_control_outputs[0]);
        copter.motors->rc_write(AP_MOTORS_MOT_2, last_control_outputs[1]);
        copter.motors->rc_write(AP_MOTORS_MOT_3, last_control_outputs[2]);
        copter.motors->rc_write(AP_MOTORS_MOT_4, last_control_outputs[3]);
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}

#include "Copter.h"
#include <math.h>

#define ALT_HOLD_MS 10000 // Stay at dest_alt for this much ms
#define M1_FAULT_MS 100   // Time since M1 fault in ms

uint32_t _time_since_init;

bool alt_flag = false;
bool elapsed_flag = false;
bool m1_fault = false;
bool fault_fixed = false;

uint16_t last_control_outputs[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

/*
 * Init and run calls for FTC flight mode
 */

// ftc_1_init - initialise FTC controller
bool ModeFTC1::init(bool ignore_checks)
{
    if (!ignore_checks)
    {
        if (!AP::ahrs().home_is_set())
        {
            return false;
        }
    }

    // check board has initialised
    if (!copter.position_ok())
    {
        return false;
    }
    _time_since_init = AP_HAL::millis();
    return true;
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
    if (!copter.motors->armed())
    {
        copter.motors->armed(true);
        // copter.flightmode->do_user_takeoff(1, true);
        do_user_takeoff(10, true);
    }

    // AP_Mission::Mission_Command cmd = {};

    // if the mission is empty save a takeoff command
    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
    // cmd.id = MAV_CMD_NAV_TAKEOFF;
    // cmd.content.location.alt = MAX(copter.current_loc.alt, 100);
    // ModeAuto->do_takeoff(cmd);
    // ModeAuto::do_takeoff(cmd);
    // takeoff_start(cmd.content.location);


    copter.motors->rc_write(AP_MOTORS_MOT_1, last_control_outputs[0]);
    copter.motors->rc_write(AP_MOTORS_MOT_2, last_control_outputs[1]);
    copter.motors->rc_write(AP_MOTORS_MOT_3, last_control_outputs[2]);
    copter.motors->rc_write(AP_MOTORS_MOT_4, last_control_outputs[3]);
    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
}

double round(double d)
{
    return floor(d + 0.5);
}

int ModeFTC1::map_ranges(int input_start, int input_end, int output_start, int output_end, int input)
{
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    return output_start + round(slope * (input - input_start));
}
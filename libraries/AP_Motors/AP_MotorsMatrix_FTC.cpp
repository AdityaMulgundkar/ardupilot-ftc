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
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsMatrix_FTC::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record requested frame class and type
    _active_frame_class = frame_class;
    _active_frame_type = frame_type;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

void AP_MotorsMatrix_FTC::output_to_motors()
{
    int8_t i;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // no output
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
                }
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], thrust_to_actuator(_thrust_rpyt_out[i]));
                }
            }
            break;
    }

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
}

// add_motor
void AP_MotorsMatrix_FTC::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor)
{
    if (initialised_ok()) {
        // do not allow motors to be set if the current frame type has init correctly
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "add_motor_raw, num: %d, r: %f, p: %f, ord: %d, th: %f", motor_num, roll_fac, pitch_fac, testing_order, throttle_factor);

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

        // enable motor
        motor_enabled[motor_num] = true;

        // set roll, pitch, yaw and throttle factors
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;
        _throttle_factor[motor_num] = throttle_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

void AP_MotorsMatrix_FTC::add_motors(const struct MotorDef *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &motor = motors[i];
        add_motor(i, motor.angle_degrees, motor.yaw_factor, motor.testing_order);
    }
}

void AP_MotorsMatrix_FTC::add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &m = motors[i];
        add_motor_raw(i, m.roll_fac, m.pitch_fac, m.yaw_fac, m.testing_order);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix_FTC::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix_FTC::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix_FTC::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0.0f;
        _pitch_factor[motor_num] = 0.0f;
        _yaw_factor[motor_num] = 0.0f;
        _throttle_factor[motor_num] = 0.0f;
    }
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix_FTC::re_enable_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = true;
        _throttle_factor[motor_num] = 1.0f;
    }
}

void AP_MotorsMatrix_FTC::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }
    set_initialised_ok(false);
    bool success = true;
    _frame_class_string = "HEXA";
    _mav_type = MAV_TYPE_HEXAROTOR;
    _frame_type_string = "X";
    
    static const AP_MotorsMatrix_FTC::MotorDef motors[] {
        {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
        {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
        {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
        { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
    };
    add_motors(motors, ARRAY_SIZE(motors));

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    set_initialised_ok(success);
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
// normalizes throttle factors so max value is 1 and no value is less than 0
void AP_MotorsMatrix_FTC::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;
    float throttle_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            roll_fac = MAX(roll_fac,fabsf(_roll_factor[i]));
            pitch_fac = MAX(pitch_fac,fabsf(_pitch_factor[i]));
            yaw_fac = MAX(yaw_fac,fabsf(_yaw_factor[i]));
            throttle_fac = MAX(throttle_fac,MAX(0.0f,_throttle_factor[i]));
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
            if (!is_zero(throttle_fac)) {
                _throttle_factor[i] = MAX(0.0f,_throttle_factor[i] / throttle_fac);
            }
        }
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix_FTC::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   throttle_thrust_max;        // throttle thrust maximum value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   yaw_allowed = 1.0f;         // amount of yaw we can fit in
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    throttle_thrust_max = _thrust_boost_ratio + (1.0f - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // calculate the highest allowed average thrust that will provide maximum control range
    throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // Under the motor lost condition we remove the highest motor output from our calculations and let that motor go greater than 1.0
    // To ensure control and maximum righting performance Hex and Octo have some optimal settings that should be used
    // Y6               : MOT_YAW_HEADROOM = 350, ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.375, ATC_RAT_PIT_IMAX = 0.375, ATC_RAT_YAW_IMAX = 0.375
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.75,  ATC_RAT_PIT_IMAX = 0.75,  ATC_RAT_YAW_IMAX = 0.375
    // Usable minimums below may result in attitude offsets when motors are lost. Hex aircraft are only marginal and must be handles with care
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.25,  ATC_RAT_PIT_IMAX = 0.25,  ATC_RAT_YAW_IMAX = 0.25
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.5,   ATC_RAT_PIT_IMAX = 0.5,   ATC_RAT_YAW_IMAX = 0.25
    // Quads cannot make use of motor loss handling because it doesn't have enough degrees of freedom.

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    float rp_low = 1.0f;    // lowest thrust value
    float rp_high = -1.0f;  // highest thrust value
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // calculate the thrust outputs for roll and pitch
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            // record lowest roll + pitch command
            if (_thrust_rpyt_out[i] < rp_low) {
                rp_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch command
            if (_thrust_rpyt_out[i] > rp_high && (!_thrust_boost || i != _motor_lost_index)) {
                rp_high = _thrust_rpyt_out[i];
            }

            // Check the maximum yaw control that can be used on this channel
            // Exclude any lost motors if thrust boost is enabled
            if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index)){
                if (is_positive(yaw_thrust * _yaw_factor[i])) {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]), 0.0f)/_yaw_factor[i]));
                } else {
                    yaw_allowed = MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[i], 0.0f)/_yaw_factor[i]));
                }
            }
        }
    }

    // calculate the maximum yaw control that can be used
    // todo: make _yaw_headroom 0 to 1
    float yaw_allowed_min = (float)_yaw_headroom * 0.001f;

    // increase yaw headroom to 50% if thrust boost enabled
    yaw_allowed_min = _thrust_boost_ratio * 0.5f + (1.0f - _thrust_boost_ratio) * yaw_allowed_min;

    // Let yaw access minimum amount of head room
    yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);

    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost && motor_enabled[_motor_lost_index]) {
        // record highest roll + pitch command
        if (_thrust_rpyt_out[_motor_lost_index] > rp_high) {
            rp_high = _thrust_boost_ratio * rp_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }

        // Check the maximum yaw control that can be used on this channel
        // Exclude any lost motors if thrust boost is enabled
        if (!is_zero(_yaw_factor[_motor_lost_index])){
            if (is_positive(yaw_thrust * _yaw_factor[_motor_lost_index])) {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(1.0f - (throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index]), 0.0f)/_yaw_factor[_motor_lost_index]));
            } else {
                yaw_allowed = _thrust_boost_ratio * yaw_allowed + (1.0f - _thrust_boost_ratio) * MIN(yaw_allowed, fabsf(MAX(throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index], 0.0f)/_yaw_factor[_motor_lost_index]));
            }
        }
    }

    if (fabsf(yaw_thrust) > yaw_allowed) {
        // not all commanded yaw can be used
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw control to thrust outputs
    float rpy_low = 1.0f;   // lowest thrust value
    float rpy_high = -1.0f; // highest thrust value
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll + pitch + yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch + yaw command
            // Exclude any lost motors if thrust boost is enabled
            if (_thrust_rpyt_out[i] > rpy_high && (!_thrust_boost || i != _motor_lost_index)) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }
    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost) {
        // record highest roll + pitch + yaw command
        if (_thrust_rpyt_out[_motor_lost_index] > rpy_high && motor_enabled[_motor_lost_index]) {
            rpy_high = _thrust_boost_ratio * rpy_high + (1.0f - _thrust_boost_ratio) * _thrust_rpyt_out[_motor_lost_index];
        }
    }

    // calculate any scaling needed to make the combined thrust outputs fit within the output range
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0) {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // calculate how close the motors can come to the desired throttle
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // Full range is being used by roll, pitch, and yaw.
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < 0.0f) {
            // Throttle can't be reduced to desired value
            // todo: add lower limit flag and ensure it is handled correctly in altitude controller
            thr_adj = 0.0f;
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
            // Throttle can't be increased to desired value
            thr_adj = 1.0f - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i]) + (rpy_scale * _thrust_rpyt_out[i]);
        }
    }

    // determine throttle thrust for harmonic notch
    // compensation_gain can never be zero
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // check for failed motor
    check_for_failed_motor(throttle_thrust_best_plus_adj);
}

// check for failed motor
//   should be run immediately after output_armed_stabilizing
//   first argument is the sum of:
//      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
//      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
//   records filtered motor output values in _thrust_rpyt_out_filt array
//   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
//   sets _motor_lost_index to index of failed motor
void AP_MotorsMatrix_FTC::check_for_failed_motor(float throttle_thrust_best_plus_adj)
{
    // record filtered and scaled thrust output for motor loss monitoring purposes
    float alpha = 1.0f / (1.0f + _loop_rate * 0.5f);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
        }
    }

    float rpyt_high = 0.0f;
    float rpyt_sum = 0.0f;
    uint8_t number_motors = 0.0f;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            number_motors += 1;
            rpyt_sum += _thrust_rpyt_out_filt[i];
            // record highest filtered thrust command
            if (_thrust_rpyt_out_filt[i] > rpyt_high) {
                rpyt_high = _thrust_rpyt_out_filt[i];
                // hold motor lost index constant while thrust boost is active
                if (!_thrust_boost) {
                    _motor_lost_index = i;
                }
            }
        }
    }

    float thrust_balance = 1.0f;
    if (rpyt_sum > 0.1f) {
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    }
    // ensure thrust balance does not activate for multirotors with less than 6 motors
    if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced) {
        _thrust_balanced = false;
    }
    if (thrust_balance <= 1.25f && !_thrust_balanced) {
        _thrust_balanced = true;
    }

    // check to see if thrust boost is using more throttle than _throttle_thrust_max
    if ((_throttle_thrust_max * get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced) {
        _thrust_boost = false;
    }
}

// output roll/pitch/yaw/thrust
void AP_MotorsMatrix_FTC::output_rpyt(void)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_roll_out, _roll_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_pitch_out, _pitch_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out, _yaw_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_thrust_out, get_throttle() * 1000);
}

// singleton instance
AP_MotorsMatrix_FTC *AP_MotorsMatrix_FTC::_singleton;
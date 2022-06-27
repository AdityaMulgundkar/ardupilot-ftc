#pragma once

#include "AP_MotorsMatrix.h"

class AP_MotorsMatrix_FTC : public AP_MotorsMatrix {
public:
    // Constructor
    AP_MotorsMatrix_FTC(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix_FTC must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_FTC *get_singleton() {
        return _singleton;
    }

    // base class method must not be used
    virtual void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void output_to_motors() override;

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float get_roll_factor(uint8_t i) override { return _roll_factor[i]; }

    // return the pitch factor of any motor
    float get_pitch_factor(uint8_t i) override { return _pitch_factor[i]; }

    // add_motor using raw roll, pitch, throttle and yaw factors
    void add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor = 1.0f);

    // remove_motor
    void remove_motor(int8_t motor_num);

    void set_roll_factor(float rolls[6]) {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            _roll_factor[i] = rolls[i];
        }
    }

    void set_pitch_factor(float pitches[6]) {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            _pitch_factor[i] = pitches[i];
        }
        
    }

protected:
    const char* _get_frame_string() const override { return "FTC"; }

    // configures the motors for the defined frame_class and frame_type
    virtual void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    void normalise_rpy_factors();

    float _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to throttle 0~1
    uint8_t _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence

private:
    static AP_MotorsMatrix_FTC *_singleton;
};
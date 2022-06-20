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
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    // add a motor and give its testing order
    bool add_motor(uint8_t motor_num, uint8_t testing_order);

    // output - sends commands to the motors
    void output_to_motors() override;

protected:

    const char* _get_frame_string() const override { return "FTC"; }

private:
    static AP_MotorsMatrix_FTC *_singleton;
};
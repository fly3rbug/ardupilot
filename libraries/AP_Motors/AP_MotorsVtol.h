// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsVtol.h
/// @brief	Motor control class for Vtolcopters

#ifndef __AP_MOTORS_VTOL_H__
#define __AP_MOTORS_VTOL_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

#define VTOL_CH_HORIZONTAL_ROLL    CH_5
#define VTOL_CH_HORIZONTAL_PITCH   CH_6
#define VTOL_CH_YAW                CH_7
#define VTOL_CH_TRANSITION         CH_8

#define VTOL_NUMBERS_OF_SERVOS         4
#define VTOL_SERVOS_OFFSET             4

#define VTOL_VERTICAL                  0
#define VTOL_HORIZONTAL                1


#define VTOL_TRANSITION_IN_SECONDS 5.0

/// @class      AP_MotorsVtol
class AP_MotorsVtol : public AP_Motors {
public:

    /// Constructor
    AP_MotorsVtol(RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, RC_Channel& rc_tail, RC_Channel& rc_horizontal_roll, RC_Channel& rc_horizontal_pitch, RC_Channel& rc_transition, uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, loop_rate, speed_hz),
        _rc_tail(rc_tail), _rc_horizontal_roll(rc_horizontal_roll), _rc_horizontal_pitch(rc_horizontal_pitch), _rc_transition(rc_transition)
    {
        _transition_counter = 0;
        _transition_max = VTOL_TRANSITION_IN_SECONDS * loop_rate;
        _flight_mode = VTOL_VERTICAL;
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

    virtual void        set_flight_mode(uint8_t mode);

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    virtual void        output_vertical_min();
    virtual void        output_horizontal_min();

    virtual void        calc_yaw_and_transition(int16_t servo_out[], float transition_factor);
    virtual void        calc_horizontal_roll_pitch(int16_t servo_out[], float transition_factor);

    virtual void        calc_vertical_roll_pitch(int16_t motor_out[], float transition_factor);
    virtual void        calc_throttle(int16_t motor_out[]);

    virtual void        output_motors(int16_t motor_out[]);
    virtual void        output_servos(int16_t servo_out[]);

    virtual void        set_throttle_limits();
    virtual float       calc_transition_factor();

    uint16_t            _transition_counter;
    uint16_t            _transition_max;

    uint8_t             _flight_mode;

    RC_Channel&         _rc_tail;       // REV parameter used from this channel to determine direction of tail servo movement
    RC_Channel&         _rc_horizontal_roll;
    RC_Channel&         _rc_horizontal_pitch;
    RC_Channel&         _rc_transition;
};

#endif  // AP_MOTORSVtol

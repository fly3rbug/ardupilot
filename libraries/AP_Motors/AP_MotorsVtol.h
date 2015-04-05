// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsVtol.h
/// @brief	Motor control class for Vtolcopters

#ifndef __AP_MOTORS_VTOL_H__
#define __AP_MOTORS_VTOL_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// channels for the control surfaces and transition
#define VTOL_CH_HORIZONTAL_ROLL         AP_MOTORS_MOT_5
#define VTOL_CH_HORIZONTAL_PITCH        AP_MOTORS_MOT_6
#define VTOL_CH_VERTICAL_PITCH_FRONT    AP_MOTORS_MOT_1
#define VTOL_CH_VERTICAL_ROLL_RIGHT     AP_MOTORS_MOT_2
#define VTOL_CH_VERTICAL_ROLL_LEFT      AP_MOTORS_MOT_4
#define VTOL_CH_YAW                     AP_MOTORS_MOT_7
#define VTOL_CH_THROTTLE                AP_MOTORS_MOT_3
#define VTOL_CH_TRANSITION              AP_MOTORS_MOT_8

// VTOL modes
#define VTOL_VERTICAL_MODE              0
#define VTOL_HORIZONTAL_MODE            1

// Internal declarations
#define VTOL_NUMBERS_OF_MOTORS          4
#define VTOL_NUMBERS_OF_SERVOS          4
#define VTOL_SERVOS_OFFSET              4

#define VTOL_TRANSITION_TIME_DEFAULT            5.0f
#define VTOL_TRANSITION_CURVE_LENGTH_DEFAULT    0.75f
#define VTOL_TRANSITION_CURVE_EXPONENT_DEFAULT  4.0f

/// @class      AP_MotorsVtol
class AP_MotorsVtol : public AP_Motors {
public:

    /// Constructor
    AP_MotorsVtol(
            RC_Channel& rc_roll,
            RC_Channel& rc_pitch,
            RC_Channel& rc_throttle,
            RC_Channel& rc_yaw,
            RC_Channel& rc_tail,
            RC_Channel& rc_horizontal_roll,
            RC_Channel& rc_horizontal_pitch,
            RC_Channel& rc_transition,
            uint16_t loop_rate,
            uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(
                rc_roll,
                rc_pitch,
                rc_throttle,
                rc_yaw,
                loop_rate,
                speed_hz),
                _rc_tail(rc_tail),
                _rc_horizontal_roll(rc_horizontal_roll),
                _rc_horizontal_pitch(rc_horizontal_pitch),
                _rc_transition(rc_transition)
    {
        AP_Param::setup_object_defaults(this, var_info);
        //_vtol_transition_time = vtol_transition_time;
        _transition_counter = 0;
        _flight_mode = VTOL_VERTICAL_MODE;
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

    virtual float       get_transition_progress();

    virtual bool        is_transition_to_horizontal_mode_done();
    virtual bool        is_transition_to_vertical_mode_done();
    virtual bool        is_transitioning();

    void                set_vtol_mode(uint8_t mode);
    uint8_t             get_vtol_mode();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    virtual void        output_vertical_min();
    virtual void        output_horizontal_min();

    virtual void        calc_yaw_and_transition(int16_t motor_out[], float transition_factor);
    virtual void        calc_horizontal_roll_pitch(int16_t motor_out[], float transition_factor);

    virtual void        calc_vertical_roll_pitch(int16_t motor_out[], float transition_factor);
    virtual void        calc_throttle(int16_t motor_out[]);

    virtual void        output_motors(int16_t motor_out[]);

    virtual void        set_throttle_limits();

    virtual float       calc_transition_control_curve_values(float x);

    AP_Float            _vtol_transition_time;  // vtol transition time in seconds
    AP_Float            _vtol_transition_curve_length;  // vtol transition curve length in 0 to 1 representing 0 to 100%
    AP_Float            _vtol_transition_curve_exponent;  // vtol transition exponent for the curve calculation
    uint16_t            _transition_counter;
    uint8_t             _flight_mode;

    RC_Channel&         _rc_tail;       // REV parameter used from this channel to determine direction of tail servo movement
    RC_Channel&         _rc_horizontal_roll;
    RC_Channel&         _rc_horizontal_pitch;
    RC_Channel&         _rc_transition;
};

#endif  // AP_MOTORSVtol

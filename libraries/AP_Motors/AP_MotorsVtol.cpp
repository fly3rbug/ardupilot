// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       AP_MotorsVtol.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_MotorsVtol.h"
#include <math.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsVtol::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO
    // 1,2,3 were used by throttle curve

    // @Param: TRANS_TIME
    // @DisplayName: VTOL transition time
    // @Description: VTOL transition time in seconds
    // @Range: 0.0 30.0
    // @Increment: 0.1
    // @Units: Seconds
    // @User: Advanced
    AP_GROUPINFO("TRANS_TIME", 5, AP_MotorsVtol, _vtol_transition_time, VTOL_TRANSITION_TIME_DEFAULT),

    // @Param: TR_CRV_LEN
    // @DisplayName: VTOL transition curve length
    // @Description: Sets the length of the transition curve. Value between 0 and 1 represends 0 to 100% of the transition time.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @Units: percentage
    // @User: Advanced
    AP_GROUPINFO("TR_CRV_LEN", 6, AP_MotorsVtol, _vtol_transition_curve_length, VTOL_TRANSITION_CURVE_LENGTH_DEFAULT),

    // @Param: TR_CRV_EXP
    // @DisplayName: VTOL transition curve exponent
    // @Description: VTOL transition curve exponent used for calculating the curve.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @Units: Exponent
    // @User: Advanced
    AP_GROUPINFO("TR_CRV_EXP", 7, AP_MotorsVtol, _vtol_transition_curve_exponent, VTOL_TRANSITION_CURVE_EXPONENT_DEFAULT),

    AP_GROUPEND
};

// init
void AP_MotorsVtol::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[VTOL_CH_VERTICAL_PITCH_FRONT] = true;
    motor_enabled[VTOL_CH_VERTICAL_ROLL_RIGHT] = true;
    motor_enabled[VTOL_CH_THROTTLE] = true;
    motor_enabled[VTOL_CH_VERTICAL_ROLL_LEFT] = true;

    RC_Channel_aux::disable_aux_channel(VTOL_CH_YAW);
    RC_Channel_aux::disable_aux_channel(VTOL_CH_HORIZONTAL_PITCH);
    RC_Channel_aux::disable_aux_channel(VTOL_CH_HORIZONTAL_ROLL);
    RC_Channel_aux::disable_aux_channel(VTOL_CH_TRANSITION);
}

// set update rate to motors - a value in hertz
void AP_MotorsVtol::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 4 motors (but not the servo channels)
    uint32_t mask = 
	    1U << pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_PITCH_FRONT]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_RIGHT]) |
        1U << pgm_read_byte(&_motor_to_channel_map[VTOL_CH_THROTTLE]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_LEFT]);
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsVtol::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_PITCH_FRONT]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_RIGHT]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_THROTTLE]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_LEFT]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_YAW]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_PITCH]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_ROLL]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_TRANSITION]));
}

// output_min - sends minimum values out to the motors
void AP_MotorsVtol::output_min()
{
    output_vertical_min();
    output_horizontal_min();
}

void AP_MotorsVtol::output_vertical_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    // send minimum value to each motor
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_PITCH_FRONT]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_RIGHT]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_THROTTLE]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_LEFT]), _rc_throttle.radio_min);
}

void AP_MotorsVtol::output_horizontal_min()
{
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_YAW]), _rc_yaw.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_PITCH]), _rc_pitch.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_ROLL]), _rc_roll.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_TRANSITION]), _rc_transition.radio_min);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsVtol::get_motor_mask()
{
    // Vtol copter uses channels 1,2,3,4 and the others
    return (
            1U << 0 |
            1U << 1 |
            1U << 2 |
            1U << 3 |
            1U << VTOL_CH_YAW |
            1U << VTOL_CH_HORIZONTAL_PITCH |
            1U << VTOL_CH_HORIZONTAL_ROLL |
            1U << VTOL_CH_TRANSITION
            );
}

void AP_MotorsVtol::set_vtol_mode(uint8_t mode)
{
    _flight_mode = mode;
}

uint8_t AP_MotorsVtol::get_vtol_mode()
{
    return _flight_mode;
}

void AP_MotorsVtol::set_throttle_limits()
{
    // initialize lower limit flag
    limit.throttle_lower = false;

    // Throttle is 0 to 1000 only
    if (_rc_throttle.servo_out <= 0) {
        _rc_throttle.servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle.servo_out >= _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // check if throttle is below limit
    if (_rc_throttle.servo_out <= _min_throttle) {
        limit.throttle_lower = true;
    }
}

bool AP_MotorsVtol::is_transition_to_horizontal_mode_done()
{
    return (_transition_counter == (_vtol_transition_time * _loop_rate * 2));
}

bool AP_MotorsVtol::is_transition_to_vertical_mode_done()
{
    return _transition_counter == 0;
}

bool AP_MotorsVtol::is_transitioning()
{
    return (_transition_counter == 0 || is_transition_to_horizontal_mode_done());
}

float AP_MotorsVtol::get_transition_progress()
{
    if (_flight_mode == VTOL_VERTICAL_MODE)
    {
        if (_transition_counter > 0)
        {
            _transition_counter--;
        }
    }
    else if (_flight_mode == VTOL_HORIZONTAL_MODE)
    {
        if (_transition_counter < (_vtol_transition_time * _loop_rate * 2))
        {
            _transition_counter++;
        }
    }

    return (float)_transition_counter / (_vtol_transition_time * _loop_rate * 2);
}

float AP_MotorsVtol::calc_transition_control_curve_values(float x)
{
    float value = 1 - pow( fabs(x) / _vtol_transition_curve_length, _vtol_transition_curve_exponent);

    value = max(value, 0.0f);
    value = min(value, 1.0f);

    return value;
}

void AP_MotorsVtol::calc_throttle(int16_t motor_out[])
{
    motor_out[VTOL_CH_THROTTLE] += _rc_throttle.radio_out;
}

void AP_MotorsVtol::calc_vertical_roll_pitch(int16_t motor_out[], float transition_factor)
{
    float transition_curve = calc_transition_control_curve_values(transition_factor);

    motor_out[VTOL_CH_THROTTLE] += _rc_pitch.pwm_out * transition_curve;

    if (!is_transition_to_horizontal_mode_done())
    {
        // calc motor output
        motor_out[VTOL_CH_VERTICAL_ROLL_LEFT] = (_rc_throttle.radio_out + _rc_roll.pwm_out) * transition_curve;
        motor_out[VTOL_CH_VERTICAL_ROLL_RIGHT] = (_rc_throttle.radio_out - _rc_roll.pwm_out) * transition_curve;
        motor_out[VTOL_CH_VERTICAL_PITCH_FRONT] = (_rc_throttle.radio_out - _rc_pitch.pwm_out) * transition_curve;

        // If motor 1 is at max then lower opposite motor
        if(motor_out[VTOL_CH_VERTICAL_PITCH_FRONT] > _rc_throttle.radio_max)
        {
            motor_out[VTOL_CH_THROTTLE] -= (motor_out[VTOL_CH_VERTICAL_PITCH_FRONT] - _rc_throttle.radio_max);
            motor_out[VTOL_CH_VERTICAL_PITCH_FRONT] = _rc_throttle.radio_max;
        }

        // If motor 2 is at max then lower opposite motor
        if(motor_out[VTOL_CH_VERTICAL_ROLL_RIGHT] > _rc_throttle.radio_max)
        {
            motor_out[VTOL_CH_VERTICAL_ROLL_LEFT] -= (motor_out[VTOL_CH_VERTICAL_ROLL_RIGHT] - _rc_throttle.radio_max);
            motor_out[VTOL_CH_VERTICAL_ROLL_RIGHT] = _rc_throttle.radio_max;
        }

        // If motor 4 is at max then lower opposite motor
        if(motor_out[VTOL_CH_VERTICAL_ROLL_LEFT] > _rc_throttle.radio_max)
        {
            motor_out[VTOL_CH_VERTICAL_ROLL_RIGHT] -= (motor_out[VTOL_CH_VERTICAL_ROLL_LEFT] - _rc_throttle.radio_max);
            motor_out[VTOL_CH_VERTICAL_ROLL_LEFT] = _rc_throttle.radio_max;
        }

        // If motor 3 is at max then lower opposite motor
        if(motor_out[VTOL_CH_THROTTLE] > _rc_throttle.radio_max)
        {
            motor_out[VTOL_CH_VERTICAL_PITCH_FRONT] -= (motor_out[VTOL_CH_THROTTLE] - _rc_throttle.radio_max);
            motor_out[VTOL_CH_THROTTLE] = _rc_throttle.radio_max;
        }
    }
}

void AP_MotorsVtol::calc_horizontal_roll_pitch(int16_t motor_out[], float transition_factor)
{
    float transition_curve = calc_transition_control_curve_values((transition_factor * -1) + 1);

    motor_out[VTOL_CH_HORIZONTAL_ROLL] = _rc_roll.radio_trim + (_rc_roll.pwm_out * transition_curve);
    motor_out[VTOL_CH_HORIZONTAL_PITCH] = _rc_pitch.radio_trim + (_rc_pitch.pwm_out * transition_curve);
}

void AP_MotorsVtol::calc_yaw_and_transition(int16_t motor_out[], float transition_factor)
{
    motor_out[VTOL_CH_YAW] = _rc_yaw.radio_out;
    motor_out[VTOL_CH_TRANSITION] = _rc_transition.radio_max - ((_rc_transition.radio_max - _rc_transition.radio_min) * transition_factor);
}

void AP_MotorsVtol::output_motors(int16_t motor_out[])
{
    int16_t out_min = _rc_throttle.radio_min + _min_throttle;
    int16_t out_max = _rc_throttle.radio_max;

    if (_flags.armed ) {
        for (uint8_t i = 0; i < VTOL_NUMBERS_OF_MOTORS + VTOL_NUMBERS_OF_SERVOS; i++)
        {
            if (i < VTOL_SERVOS_OFFSET)
            {
                if (is_transition_to_horizontal_mode_done() && i != VTOL_CH_THROTTLE)
                {
                    motor_out[i] = _rc_throttle.radio_min;
                }
                else
                {
                    motor_out[i] = max(motor_out[i], out_min);
                    motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min, out_max);
                }
            }

            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), motor_out[i]);
        }
    }
    else
    {
        output_vertical_min();
        for (uint8_t i = VTOL_SERVOS_OFFSET; i < VTOL_NUMBERS_OF_SERVOS + VTOL_SERVOS_OFFSET; i++)
        {
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), motor_out[i]);
        }
    }
}

// output_armed - sends commands to the motors
void AP_MotorsVtol::output_armed()
{
    int16_t motor_out[VTOL_NUMBERS_OF_MOTORS + VTOL_NUMBERS_OF_SERVOS];
    float transition_factor = get_transition_progress();

    for (uint8_t i = 0; i < VTOL_NUMBERS_OF_MOTORS + VTOL_NUMBERS_OF_SERVOS; i++)
    {
        motor_out[i] = 0;
    }

    set_throttle_limits();

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_yaw.calc_pwm();
    _rc_throttle.calc_pwm();

    calc_throttle(motor_out);
    calc_vertical_roll_pitch(motor_out, transition_factor);
    calc_horizontal_roll_pitch(motor_out, transition_factor);
    calc_yaw_and_transition(motor_out, transition_factor);

    output_motors(motor_out);
}

// output_disarmed - sends commands to the motors
void AP_MotorsVtol::output_disarmed()
{
    float transition_factor = get_transition_progress();

    int16_t motor_out[VTOL_NUMBERS_OF_MOTORS + VTOL_NUMBERS_OF_SERVOS];
    for (uint8_t i = 0; i < VTOL_NUMBERS_OF_MOTORS + VTOL_NUMBERS_OF_SERVOS; i++)
    {
        motor_out[i] = 0;
    }

    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_yaw.calc_pwm();

    calc_horizontal_roll_pitch(motor_out, transition_factor);
    calc_yaw_and_transition(motor_out, transition_factor);
    output_motors(motor_out);
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsVtol::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!_flags.armed) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_PITCH_FRONT]), pwm);
            break;
        case 2:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_RIGHT]), pwm);
            break;
        case 3:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_THROTTLE]), pwm);
            break;
        case 4:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_VERTICAL_ROLL_LEFT]), pwm);
            break;
        case 5:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_ROLL]), pwm);
            break;
        case 6:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_HORIZONTAL_PITCH]), pwm);
            break;
        case 7:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_YAW]), pwm);
            break;
        case 8:
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[VTOL_CH_TRANSITION]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}

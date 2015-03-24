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

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsVtol::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // disable CH7 from being used as an aux output (i.e. for camera gimbal, etc)
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
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]) |
        1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]) |
	    1U << pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]);
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsVtol::enable()
{
    // enable output channels
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));
    hal.rcout->enable_ch(VTOL_CH_YAW);
    hal.rcout->enable_ch(VTOL_CH_HORIZONTAL_PITCH);
    hal.rcout->enable_ch(VTOL_CH_HORIZONTAL_ROLL);
    hal.rcout->enable_ch(VTOL_CH_TRANSITION);
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
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _rc_throttle.radio_min);
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

void AP_MotorsVtol::set_flight_mode(uint8_t mode)
{
    _flight_mode = mode;
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

    // Vtolcopters limit throttle to 90%
    // To-Do: implement improved stability patch and remove this limit
    if (_flight_mode == VTOL_VERTICAL) {
        if (_rc_throttle.servo_out > 900) {
            _rc_throttle.servo_out = 900;
            limit.throttle_upper = true;
        }
    }

    // check if throttle is below limit
    if (_rc_throttle.servo_out <= _min_throttle) {
        limit.throttle_lower = true;
    }
}

float AP_MotorsVtol::calc_transition_factor()
{
    if (_flight_mode == VTOL_VERTICAL)
    {
        if (_transition_counter > 0)
        {
            _transition_counter--;
        }
    }
    else if (_flight_mode == VTOL_HORIZONTAL)
    {
        if (_transition_counter < _transition_max)
        {
            _transition_counter++;
        }
    }

    return (float)_transition_counter / _transition_max;
}

void AP_MotorsVtol::calc_throttle(int16_t motor_out[])
{
    motor_out[AP_MOTORS_MOT_3] += _rc_throttle.radio_out;
}

void AP_MotorsVtol::calc_vertical_roll_pitch(int16_t motor_out[], float transition_factor)
{
    transition_factor = (transition_factor * -1) + 1;

    motor_out[AP_MOTORS_MOT_3] += _rc_pitch.pwm_out * transition_factor;

    if (_transition_counter != _transition_max)
    {
        //left
        motor_out[AP_MOTORS_MOT_4] = (_rc_throttle.radio_out + _rc_roll.pwm_out) * transition_factor;
        //right
        motor_out[AP_MOTORS_MOT_2] = (_rc_throttle.radio_out - _rc_roll.pwm_out) * transition_factor;
        // front
        motor_out[AP_MOTORS_MOT_1] = (_rc_throttle.radio_out - _rc_pitch.pwm_out) * transition_factor;

        // If motor 1 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_1] > _rc_throttle.radio_max)
        {
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_1] - _rc_throttle.radio_max);
            motor_out[AP_MOTORS_MOT_1] = _rc_throttle.radio_max;
        }

        // If motor 2 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_2] > _rc_throttle.radio_max)
        {
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_2] - _rc_throttle.radio_max);
            motor_out[AP_MOTORS_MOT_2] = _rc_throttle.radio_max;
        }

        // If motor 4 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_4] > _rc_throttle.radio_max)
        {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_4] - _rc_throttle.radio_max);
            motor_out[AP_MOTORS_MOT_4] = _rc_throttle.radio_max;
        }

        // If motor 3 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_3] > _rc_throttle.radio_max)
        {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_3] - _rc_throttle.radio_max);
            motor_out[AP_MOTORS_MOT_3] = _rc_throttle.radio_max;
        }
    }
}

void AP_MotorsVtol::calc_horizontal_roll_pitch(int16_t servo_out[], float transition_factor)
{
    servo_out[VTOL_CH_HORIZONTAL_ROLL - VTOL_SERVOS_OFFSET] = _rc_roll.radio_trim + (_rc_roll.pwm_out * transition_factor);
    servo_out[VTOL_CH_HORIZONTAL_PITCH - VTOL_SERVOS_OFFSET] = _rc_pitch.radio_trim + (_rc_pitch.pwm_out * transition_factor);
}

void AP_MotorsVtol::calc_yaw_and_transition(int16_t servo_out[], float transition_factor)
{
    servo_out[VTOL_CH_YAW - VTOL_SERVOS_OFFSET] = _rc_yaw.radio_out;
    servo_out[VTOL_CH_TRANSITION - VTOL_SERVOS_OFFSET] = _rc_transition.radio_min + ((_rc_transition.radio_max - _rc_transition.radio_min) * transition_factor);
}

void AP_MotorsVtol::output_motors(int16_t motor_out[])
{
    int16_t out_min = _rc_throttle.radio_min + _min_throttle;
    int16_t out_max = _rc_throttle.radio_max;

    for (uint8_t i = 0; i < AP_MOTORS_MOT_4+1; i++)
    {
        motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min, out_max);
        motor_out[i] = max(motor_out[i], out_min);
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), motor_out[i]);
    }
}

void AP_MotorsVtol::output_servos(int16_t servo_out[])
{
    for (uint8_t i = 0; i < VTOL_NUMBERS_OF_SERVOS; i++)
    {
        hal.rcout->write(i + VTOL_SERVOS_OFFSET, servo_out[i]);
    }
}

// output_armed - sends commands to the motors
void AP_MotorsVtol::output_armed()
{
    int16_t motor_out[AP_MOTORS_MOT_4+1];
    int16_t servo_out[VTOL_NUMBERS_OF_SERVOS];
    float transition_factor = calc_transition_factor();

    for (uint8_t i = 0; i < AP_MOTORS_MOT_4 + 1; i++)
    {
        motor_out[i] = 0;
    }
    for (uint8_t i = 0; i < VTOL_NUMBERS_OF_SERVOS; i++)
    {
        servo_out[i] = 0;
    }


    set_throttle_limits();

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_yaw.calc_pwm();
    _rc_throttle.calc_pwm();

    calc_throttle(motor_out);
    calc_vertical_roll_pitch(motor_out, transition_factor);
    calc_horizontal_roll_pitch(servo_out, transition_factor);
    calc_yaw_and_transition(servo_out, transition_factor);

    output_motors(motor_out);
    output_servos(servo_out);
}

// output_disarmed - sends commands to the motors
void AP_MotorsVtol::output_disarmed()
{
    float transition_factor = calc_transition_factor();
    int16_t servo_out[VTOL_NUMBERS_OF_SERVOS];
    for (uint8_t i = 0; i < VTOL_NUMBERS_OF_SERVOS; i++)
    {
        servo_out[i] = 0;
    }

    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_yaw.calc_pwm();

    output_vertical_min();

    calc_horizontal_roll_pitch(servo_out, transition_factor);
    calc_yaw_and_transition(servo_out, transition_factor);
    output_servos(servo_out);
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
            // front right motor
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), pwm);
            break;
        case 2:
            // back motor
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), pwm);
            break;
        case 3:
            // back servo
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), pwm);
            break;
        case 4:
            // front left motor
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), pwm);
            break;
        case 5:
            // front left motor
            hal.rcout->write(VTOL_CH_HORIZONTAL_ROLL, pwm);
            break;
        case 6:
            // front left motor
            hal.rcout->write(VTOL_CH_HORIZONTAL_PITCH, pwm);
            break;
        case 7:
            // front left motor
            hal.rcout->write(VTOL_CH_YAW, pwm);
            break;
        case 8:
            // front left motor
            hal.rcout->write(VTOL_CH_TRANSITION, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

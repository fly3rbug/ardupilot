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
    RC_Channel_aux::disable_aux_channel(AP_MOTORS_CH_VTOL_YAW);

    RC_Channel_aux::disable_aux_channel(AP_MOTORS_CH_VTOL_HORIZONTAL_PITCH);
    RC_Channel_aux::disable_aux_channel(AP_MOTORS_CH_VTOL_HORIZONTAL_ROLL);
}

// set update rate to motors - a value in hertz
void AP_MotorsVtol::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 4 motors (but not the servo on channel 7)
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
    hal.rcout->enable_ch(AP_MOTORS_CH_VTOL_YAW);
    hal.rcout->enable_ch(AP_MOTORS_CH_VTOL_HORIZONTAL_PITCH);
    hal.rcout->enable_ch(AP_MOTORS_CH_VTOL_HORIZONTAL_ROLL);
}

// output_min - sends minimum values out to the motors
void AP_MotorsVtol::output_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    // send minimum value to each motor
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _rc_throttle.radio_min);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_CH_VTOL_YAW]), _rc_yaw.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_CH_VTOL_HORIZONTAL_PITCH]), _rc_pitch.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_CH_VTOL_HORIZONTAL_ROLL]), _rc_roll.radio_trim);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsVtol::get_motor_mask()
{
    // Vtol copter uses channels 1,2,3,4 and the others
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_CH_VTOL_YAW | 1U << AP_MOTORS_CH_VTOL_HORIZONTAL_PITCH | 1U << AP_MOTORS_CH_VTOL_HORIZONTAL_ROLL);
}

void AP_MotorsVtol::set_flight_mode(uint8_t mode)
{
    _flight_mode = mode;
}

// output_armed - sends commands to the motors
void AP_MotorsVtol::output_armed()
{
    int16_t out_min = _rc_throttle.radio_min + _min_throttle;
    int16_t out_max = _rc_throttle.radio_max;
    int8_t number_of_motors = AP_MOTORS_MOT_4+1;

    int16_t motor_out[number_of_motors];

    //initialize motor_out
    for (uint8_t i = 0; i < number_of_motors; ++i)
    {
        motor_out[i] = 0;
    }

    float transition_factor = (float)_transition_counter / _transition_max;

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
    if (_flight_mode == AP_MOTORS_VERTICAL) {
        if (_rc_throttle.servo_out > 900) {
            _rc_throttle.servo_out = 900;
            limit.throttle_upper = true;
        }
    }

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll.calc_pwm();
    _rc_pitch.calc_pwm();
    _rc_throttle.calc_pwm();
    _rc_yaw.calc_pwm();

    // check if throttle is below limit
    if (_rc_throttle.servo_out <= _min_throttle) {
        limit.throttle_lower = true;
    }

    if (_flight_mode == AP_MOTORS_VERTICAL) {
        if (_transition_counter > 0){
            _transition_counter--;
        }
    }else if (_flight_mode == AP_MOTORS_HORIZONTAL) {
        if (_transition_counter < _transition_max){
            _transition_counter++;
        }
    }

    bool cut_vertical_motors = false;
    if (_transition_counter == _transition_max){
        cut_vertical_motors = true;
    }

    // rear
    motor_out[AP_MOTORS_MOT_3] = (_rc_throttle.radio_out + (_rc_pitch.pwm_out * ((transition_factor * -1) + 1)));

    if (!cut_vertical_motors) {
        //left
        motor_out[AP_MOTORS_MOT_4] = (_rc_throttle.radio_out + _rc_roll.pwm_out) * ((transition_factor * -1) + 1);
        //right
        motor_out[AP_MOTORS_MOT_2] = (_rc_throttle.radio_out - _rc_roll.pwm_out) * ((transition_factor * -1) + 1);
        // front
        motor_out[AP_MOTORS_MOT_1] = (_rc_throttle.radio_out - _rc_pitch.pwm_out) * ((transition_factor * -1) + 1);

        // If motor 1 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_1] > out_max) {
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_1] = out_max;
        }
        // If motor 2 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_2] > out_max) {
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_2] = out_max;
        }
        // If motor 4 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_4] > out_max) {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_4] - out_max);
            motor_out[AP_MOTORS_MOT_4] = out_max;
        }
        // If motor 3 is at max then lower opposite motor
        if(motor_out[AP_MOTORS_MOT_3] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_3] - out_max);
            motor_out[AP_MOTORS_MOT_3] = out_max;
        }
        // adjust for thrust curve and voltage scaling
        motor_out[AP_MOTORS_MOT_1] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_1], out_min, out_max);
        motor_out[AP_MOTORS_MOT_2] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_2], out_min, out_max);
        motor_out[AP_MOTORS_MOT_4] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_4], out_min, out_max);

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_1] = max(motor_out[AP_MOTORS_MOT_1],    out_min);
        motor_out[AP_MOTORS_MOT_2] = max(motor_out[AP_MOTORS_MOT_2],    out_min);
        motor_out[AP_MOTORS_MOT_4] = max(motor_out[AP_MOTORS_MOT_4],    out_min);
    }

    // adjust for thrust curve and voltage scaling
    motor_out[AP_MOTORS_MOT_3] = apply_thrust_curve_and_volt_scaling(motor_out[AP_MOTORS_MOT_3], out_min, out_max);
    motor_out[AP_MOTORS_MOT_3] = max(motor_out[AP_MOTORS_MOT_3],    out_min);


    hal.rcout->write(AP_MOTORS_CH_VTOL_HORIZONTAL_ROLL, _rc_roll.radio_trim + (_rc_roll.pwm_out * transition_factor));
    hal.rcout->write(AP_MOTORS_CH_VTOL_HORIZONTAL_PITCH, _rc_pitch.radio_trim + (_rc_pitch.pwm_out * transition_factor));


    // send output to each motor
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), motor_out[AP_MOTORS_MOT_1]);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), motor_out[AP_MOTORS_MOT_2]);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), motor_out[AP_MOTORS_MOT_3]);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), motor_out[AP_MOTORS_MOT_4]);

    // also send out to tail command (we rely on any auto pilot to have updated the rc_yaw->radio_out to the correct value)
    // note we do not save the radio_out to the motor_out array so it may not appear in the ch7out in the status screen of the mission planner
    // note: we use _rc_tail's (aka channel 7's) REV parameter to control whether the servo is reversed or not but this is a bit nonsensical.
    //       a separate servo object (including min, max settings etc) would be better or at least a separate parameter to specify the direction of the tail servo
    if( _rc_tail.get_reverse() == true ) {
        hal.rcout->write(AP_MOTORS_CH_VTOL_YAW, _rc_yaw.radio_trim - (_rc_yaw.radio_out - _rc_yaw.radio_trim));
    }else{
        hal.rcout->write(AP_MOTORS_CH_VTOL_YAW, _rc_yaw.radio_out);
    }
}

// output_disarmed - sends commands to the motors
void AP_MotorsVtol::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
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
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), pwm);
            break;
        case 3:
            // back servo
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_7]), pwm);
            break;
        case 4:
            // front left motor
            hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), pwm);
            break;
        default:
            // do nothing
            break;
    }
}

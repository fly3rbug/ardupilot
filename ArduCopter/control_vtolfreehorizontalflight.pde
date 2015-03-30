/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_vtolfreehorizontalflight.pde - init and run calls for new flight mode
 */

// vtolfreehorizontalflight_init - Initialize flight mode
static bool vtolfreehorizontalflight_init(bool ignore_checks)
{
    motors.set_vtol_mode(VTOL_HORIZONTAL_MODE);

    attitude_control.enable_pitch_rate_control(false);
    attitude_control.enable_roll_rate_control(false);
    attitude_control.enable_yaw_rate_control(false);
    return true;
}

static void vtolfreehorizontalflight_cleanup()
{
    motors.set_vtol_mode(VTOL_VERTICAL_MODE);

    attitude_control.enable_pitch_rate_control(true);
    attitude_control.enable_roll_rate_control(true);
    attitude_control.enable_yaw_rate_control(true);
}

// vtolfreehorizontalflight_run - runs the main controller
// will be called at 100hz or more
static void vtolfreehorizontalflight_run()
{

    int16_t roll_acro, pitch_acro, yaw_acro;
    int16_t roll_diff, pitch_diff, yaw_diff;
    int16_t roll_free, pitch_free, throttle_free, yaw_free;

    float transition_progress = (motors.get_transition_progress()* -1) + 1;

    Vector3f acro_values = attitude_control.rate_bf_targets();

    throttle_free = g.rc_3.control_in;

    roll_free = g.rc_1.control_in;
    pitch_free = g.rc_2.control_in;
    yaw_free = g.rc_4.control_in;

    roll_acro = attitude_control.rate_bf_to_motor_roll(acro_values.x);
    pitch_acro = attitude_control.rate_bf_to_motor_pitch(acro_values.y);
    yaw_acro = attitude_control.rate_bf_to_motor_yaw(acro_values.z);

    roll_diff = roll_free - roll_acro;
    pitch_diff = pitch_free - pitch_acro;
    yaw_diff = yaw_free - yaw_acro;


    motors.set_roll(roll_free - (roll_diff * transition_progress));
    motors.set_pitch(pitch_free - (pitch_diff * transition_progress));
    motors.set_yaw(yaw_free - (yaw_diff * transition_progress));

    motors.set_throttle(throttle_free);

    if (motors.get_vtol_mode() == VTOL_VERTICAL_MODE && motors.is_transition_to_vertical_mode_done()){
        set_mode(STABILIZE);
    }
}

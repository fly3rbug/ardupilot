/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_vtolstabilizedhorizontalflight.pde - init and run calls for the horizontal flight mode
 */

// vtolstabilizedhorizontalflight_init - Initialize flight mode
static bool vtolstabilizedhorizontalflight_init(bool ignore_checks)
{
    // set to horizontal mode to start the transition
    motors.set_vtol_mode(VTOL_HORIZONTAL_MODE);

    // disable yaw rate control because it is not used in this flight mode. Yaw is connected directly to receiver pwm.
    attitude_control.enable_pitch_rate_control(false);
    attitude_control.enable_roll_rate_control(false);
    attitude_control.enable_yaw_rate_control(false);

    return true;
}
static void vtolstabilizedhorizontalflight_cleanup()
{
    // set back the values that were changed in the init
    motors.set_vtol_mode(VTOL_VERTICAL_MODE);
    attitude_control.enable_pitch_rate_control(true);
    attitude_control.enable_roll_rate_control(true);
    attitude_control.enable_yaw_rate_control(true);
}

// vtolstabilizedhorizontalflight_run - runs the main controller
// will be called at 100hz or more
static void vtolstabilizedhorizontalflight_run()
{

    int16_t roll_stabil, pitch_stabil, yaw_stabil;
    int16_t roll_diff, pitch_diff, yaw_diff;
    int16_t roll_acro, pitch_acro, yaw_free;

    float transition_progress = motors.get_transition_progress();

    stabilize_run();

    Vector3f stabil_values = attitude_control.rate_bf_targets();

    roll_stabil = attitude_control.rate_bf_to_motor_roll(stabil_values.x);
    pitch_stabil = attitude_control.rate_bf_to_motor_pitch(stabil_values.y);
    yaw_stabil = attitude_control.rate_bf_to_motor_yaw(stabil_values.z);

    vtolstabilizedhorizontalflight_acro();

    Vector3f acro_values = attitude_control.rate_bf_targets();

    roll_acro = attitude_control.rate_bf_to_motor_roll(acro_values.x);
    pitch_acro = attitude_control.rate_bf_to_motor_pitch(acro_values.y);
    yaw_free = g.rc_4.control_in;

    roll_diff = roll_stabil - roll_acro;
    pitch_diff = pitch_stabil - pitch_acro;
    yaw_diff = yaw_stabil - yaw_free;

    motors.set_roll(roll_stabil - (roll_diff * transition_progress));
    motors.set_pitch(pitch_stabil - (pitch_diff * transition_progress));
    motors.set_yaw(yaw_stabil - (yaw_diff * transition_progress));

    if (motors.get_vtol_mode() == VTOL_VERTICAL_MODE && motors.is_transition_to_vertical_mode_done()){
        set_mode(STABILIZE);
    }
}

static void vtolstabilizedhorizontalflight_acro()
{
    float target_roll, target_pitch, target_yaw;
    int16_t pilot_throttle_scaled;

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);

    // run attitude controller
    attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false);
}

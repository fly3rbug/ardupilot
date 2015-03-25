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
    attitude_control.enable_yaw_rate_control(false);

    return true;
}
static void vtolstabilizedhorizontalflight_cleanup()
{
    // set back the values that were changed in the init
    motors.set_vtol_mode(VTOL_VERTICAL_MODE);
    attitude_control.enable_yaw_rate_control(true);
}

// vtolstabilizedhorizontalflight_run - runs the main controller
// will be called at 100hz or more
static void vtolstabilizedhorizontalflight_run()
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
    motors.set_yaw(g.rc_4.control_in);
}

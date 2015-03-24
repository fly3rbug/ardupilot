/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_vtolstabilizedhorizontalflight.pde - init and run calls for new flight mode
 */

// vtolstabilizedhorizontalflight_init - initialise flight mode
static bool vtolstabilizedhorizontalflight_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);
    attitude_control.enable_yaw_rate_control(false);

    motors.set_flight_mode(VTOL_HORIZONTAL);

    // stabilize should never be made to fail
    return true;
}
static void vtolstabilizedhorizontalflight_cleanup()
{
    motors.set_flight_mode(VTOL_VERTICAL);

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

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // run attitude controller
    attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false);
    motors.set_yaw(g.rc_4.control_in);
}

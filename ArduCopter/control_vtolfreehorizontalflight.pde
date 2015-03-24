/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_vtolfreehorizontalflight.pde - init and run calls for new flight mode
 */

// vtolfreehorizontalflight_init - initialise flight mode
static bool vtolfreehorizontalflight_init(bool ignore_checks)
{
    motors.set_flight_mode(VTOL_HORIZONTAL);

    attitude_control.enable_pitch_rate_control(false);
    attitude_control.enable_roll_rate_control(false);
    attitude_control.enable_yaw_rate_control(false);
    return true;
}

static void vtolfreehorizontalflight_cleanup()
{
    motors.set_flight_mode(VTOL_VERTICAL);

    attitude_control.enable_pitch_rate_control(true);
    attitude_control.enable_roll_rate_control(true);
    attitude_control.enable_yaw_rate_control(true);
}

// vtolfreehorizontalflight_run - runs the main controller
// will be called at 100hz or more
static void vtolfreehorizontalflight_run()
{

    motors.set_roll(g.rc_1.control_in);
    motors.set_pitch(g.rc_2.control_in);
    motors.set_throttle(g.rc_3.control_in);
    motors.set_yaw(g.rc_4.control_in);
}

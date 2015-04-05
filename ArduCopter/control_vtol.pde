/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_vtolstabilizedhorizontalflight.pde - init and run calls for the horizontal flight mode
 */

// vtolstabilizedhorizontalflight_init - Initialize flight mode
static bool vtol_init(bool ignore_checks, uint8_t mode)
{
    if (mode == STABILIZE)
    {
        motors.set_vtol_mode(VTOL_VERTICAL_MODE);
        vtol_enable_rate_control(true);

        return motors.is_transition_to_vertical_mode_done();
    }
    else if(control_mode == STABILIZE && (mode == VTOL_ACRO_FWD_FLIGHT || mode == VTOL_FREE_FWD_FLIGHT))
    {
        motors.set_vtol_mode(VTOL_HORIZONTAL_MODE);
        vtol_enable_rate_control(false);
    }

    return true;
}

static void vtol_enable_rate_control(bool ena)
{

    attitude_control.enable_pitch_rate_control(ena);
    attitude_control.enable_roll_rate_control(ena);
    attitude_control.enable_yaw_rate_control(ena);
}

static Vector3i vtol_get_control_rates()
{
    Vector3i rates;
    Vector3f acro_values = attitude_control.rate_bf_targets();

    rates.x = attitude_control.rate_bf_to_motor_roll(acro_values.x);
    rates.y = attitude_control.rate_bf_to_motor_pitch(acro_values.y);
    rates.z = attitude_control.rate_bf_to_motor_yaw(acro_values.z);

    return rates;
}

static Vector3i vtol_get_control_in()
{
    Vector3i control_in;
    control_in.x = g.rc_1.control_in;
    control_in.y = g.rc_2.control_in;
    control_in.z = g.rc_4.control_in;

    return control_in;
}

static void vtol_stabilize_hover_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    Vector3i stabil_vector;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    stabil_vector = vtol_get_control_in();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(stabil_vector.x, stabil_vector.y, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(stabil_vector.z);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle
    vtol_set_throttle();
}

static void vtol_free_fwd_run()
{
    Vector3i stabil_vector, free_vector;
    float transition_progress = (motors.get_transition_progress()* -1) + 1;

    vtol_stabilize_hover_run();
    free_vector = vtol_get_control_in();
    stabil_vector = vtol_get_control_rates();

    motors.set_roll(free_vector.x - ((free_vector.x - stabil_vector.x) * transition_progress));
    motors.set_pitch(free_vector.y - ((free_vector.y - stabil_vector.y) * transition_progress));
    motors.set_yaw(free_vector.z - ((free_vector.z - stabil_vector.z) * transition_progress));

    vtol_set_throttle();

    if (motors.get_vtol_mode() == VTOL_VERTICAL_MODE && motors.is_transition_to_vertical_mode_done()){
        set_mode(STABILIZE);
    }
}

// vtolstabilizedhorizontalflight_run - runs the main controller
// will be called at 100hz or more
static void vtol_acro_fwd_run()
{

    Vector3i acro_vector, stabil_vector;
    float transition_progress = motors.get_transition_progress();

    stabilize_run();
    stabil_vector = vtol_get_control_rates();

    vtol_acro();
    acro_vector = vtol_get_control_rates();
    acro_vector.z = g.rc_4.control_in;

    motors.set_roll(stabil_vector.x - ((stabil_vector.x - acro_vector.x) * transition_progress));
    motors.set_pitch(stabil_vector.y - ((stabil_vector.y - acro_vector.y) * transition_progress));
    motors.set_yaw(stabil_vector.z - ((stabil_vector.z - acro_vector.z) * transition_progress));

    vtol_set_throttle();

    if (motors.get_vtol_mode() == VTOL_VERTICAL_MODE && motors.is_transition_to_vertical_mode_done()){
        set_mode(STABILIZE);
    }
}

static void vtol_acro()
{
    float target_roll, target_pitch, target_yaw;

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);

    // run attitude controller
    attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
}

static void vtol_set_throttle()
{
    int16_t pilot_throttle_scaled;

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false);
}

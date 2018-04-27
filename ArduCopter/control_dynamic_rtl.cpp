#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

/*
 * Init and run calls for dynamic RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

#define DYNAMIC_RTL_TIMEOUT_MS 3000 // Timeout value to mark target position as invalid
#define DYNAMIC_RTL_DIST_MAX 5000 // Don't attempt to land more than 5km away
#define DRTL_LATENCY_FUDGE_FACTOR 0.0f

// rtl_init - initialise rtl controller
bool Copter::dynamic_rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        dynamic_rtl_build_climb_path(!failsafe.terrain);
        rtl_climb_start();
        gcs_send_text(MAV_SEVERITY_INFO, "DRTL: Climb Start");
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void Copter::dynamic_rtl_run()
{
    // check if we need to move to next state
    if (rtl_state_complete) {
        switch (rtl_state) {
        case RTL_InitialClimb:
            dynamic_rtl_return_start();
            gcs_send_text(MAV_SEVERITY_INFO, "DRTL: Return Start");
            break;
        case RTL_ReturnHome:
            dynamic_rtl_descent_start();
            gcs_send_text(MAV_SEVERITY_INFO, "DRTL: Descent Start");
            break;
        case RTL_LoiterAtHome:
            // Not used for DRTL
            break;
        case RTL_FinalDescent:
            gcs_send_text(MAV_SEVERITY_INFO, "DRTL: Land Start");
            rtl_land_start();
            break;
        case RTL_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (rtl_state) {
    case RTL_InitialClimb:
        dynamic_rtl_climb_run();
        break;

    case RTL_ReturnHome:
        dynamic_rtl_return_run();
        break;

    case RTL_LoiterAtHome:
        // Not used for DRTL
        break;

    case RTL_FinalDescent:
        dynamic_rtl_descent_run();
        break;

    case RTL_Land:
        rtl_land_run();
        break;
    }
}

// handle mavlink MSG_ID_GLOBAL_POSITION_INT messages
void Copter::dynamic_rtl_handle_msg(mavlink_message_t *msg)
{
    // skip our own messages
    if (msg->sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if (msg->sysid != g2.drtl_sysid_to_target) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO,"DRTL SYSID Invalid %u %u", (unsigned)msg->sysid, (unsigned)g2.drtl_sysid_to_target);
        return;
    }

    // decode global-position-int message
    if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        const uint32_t now = AP_HAL::millis();

        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return;
        }

        drtl_target_location.lat = packet.lat;
        drtl_target_location.lng = packet.lon;
        drtl_target_location.set_alt_cm(int32_t(packet.alt/10), Location_Class::ALT_FRAME_ABSOLUTE);   // convert millimeters to cm
        drtl_target_velocity_ned.x = packet.vx * 0.01f; // velocity north
        drtl_target_velocity_ned.y = packet.vy * 0.01f; // velocity east
        drtl_target_velocity_ned.z = packet.vz * 0.01f; // velocity down
        drtl_last_location_update_ms = now;
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            drtl_target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
            drtl_last_heading_update_ms = now;
        }
    }
}

// get target's estimated location. Lat/Long Absolute, altitude relative to the copters home
bool Copter::dynamic_rtl_get_target_location_and_velocity(Location &loc, Vector3f &vel_ned)
{

    // check for timeout
    if ((drtl_last_location_update_ms == 0) || ((AP_HAL::millis() - drtl_last_location_update_ms) > DYNAMIC_RTL_TIMEOUT_MS)) {
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - drtl_last_location_update_ms) * 0.001f + DRTL_LATENCY_FUDGE_FACTOR;

    // Update the velocity vector
    vel_ned = drtl_target_velocity_ned;
    
    // project the vehicle position
    Location_Class last_loc = drtl_target_location;
    
    // convert altitude frame to above home location (The same frame as current_loc)
    last_loc.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME);

    location_offset(last_loc, vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 10.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    
    // log lead's estimated vs reported position
    DataFlash_Class::instance()->Log_Write("DRTL",
                                          "TimeUS,Lat,Lon,Alt,VelX,VelY,VelZ,LatE,LonE,AltE",  // labels
                                          "QLLifffLLi",    // fmt
                                          AP_HAL::micros64(),
                                          drtl_target_location.lat,
                                          drtl_target_location.lng,
                                          drtl_target_location.alt,
                                          (double)drtl_target_velocity_ned.x,
                                          (double)drtl_target_velocity_ned.y,
                                          (double)drtl_target_velocity_ned.z,
                                          loc.lat,
                                          loc.lng,
                                          loc.alt
                                                    );
    return true;
}

// Get error distance vector and the target's velocity
bool Copter::dynamic_rtl_get_target_dist_and_vel(Vector3f &dist_ned, Vector3f &vel_ned)
{

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;
    if (!dynamic_rtl_get_target_location_and_velocity(target_loc, veh_vel)) {
        return false;
    }

    // calculate difference
    const Vector3f dist_vec = location_3d_diff_NED(current_loc, target_loc);

    // fail if too far
    if ((dist_vec.length() > DYNAMIC_RTL_DIST_MAX)) {
        return false;
    }

    dist_ned = dist_vec;
    vel_ned = veh_vel;
    return true;
}

// Compute the desired climb height using the logic from RTl
void Copter::dynamic_rtl_build_climb_path(bool terrain_following_allowed)
{
    // origin point is our stopping point
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->get_stopping_point_z(stopping_point);
    rtl_path.origin_point = Location_Class(stopping_point);
    rtl_path.origin_point.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME);

    // compute return target, paying special attention to choosing the correct return path altitude
    dynamic_rtl_compute_return_target(terrain_following_allowed);

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location_Class(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());
}

// compute the return target - used to determine climb height using cone logic
//   return altitude in cm above home at which vehicle should return home
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void Copter::dynamic_rtl_compute_return_target(bool terrain_following_allowed)
{
	Location target_location;
	Vector3f target_velocity;

	if (dynamic_rtl_get_target_location_and_velocity(target_location, target_velocity)) {
		rtl_path.return_target = target_location;
	} else {
		// abort if we don't have an updated position from the target
        if (!set_mode(LOITER,  MODE_REASON_INVALID_TARGET)) {
            set_mode(ALT_HOLD,  MODE_REASON_INVALID_TARGET);
        }
        gcs_send_text(MAV_SEVERITY_INFO, "DRTL abort: target invalid");
		return;
	}

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = current_loc.alt;

    // convert return-target alt (which is an absolute alt) to alt-above-home or alt-above-terrain
    if (!rtl_path.return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME)) {
        // this should never happen but just in case
        rtl_path.return_target.set_alt_cm(0, Location_Class::ALT_FRAME_ABOVE_HOME);
    }
    rtl_path.terrain_used = false;
    

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon use_terrain
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    // TODO - deal with this target_alt = MAX(target_alt, curr_alt + MAX(0, g.rtl_climb_min));
    target_alt = MAX(target_alt, MAX(g.rtl_altitude, RTL_ALT_MIN));

    // reduce climb if close to return target
    float rtl_return_dist_cm = rtl_path.return_target.get_distance(rtl_path.origin_point) * 100.0f;
    // don't allow really shallow slopes
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        target_alt = MAX(curr_alt, MIN(target_alt, MAX(rtl_return_dist_cm*g.rtl_cone_slope, curr_alt+RTL_ABS_MIN_CLIMB)));
    }

    // set returned target alt to new target_alt
    rtl_path.return_target.set_alt_cm(target_alt, rtl_path.terrain_used ? Location_Class::ALT_FRAME_ABOVE_TERRAIN : Location_Class::ALT_FRAME_ABOVE_HOME);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        if (rtl_path.return_target.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, target_alt)) {
            float fence_alt = fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // reduce target alt to the fence alt
                rtl_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // ensure we do not descend
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt);
}

// rtl_climb_run - implements the initial climb of DRTL which relies on the wp controller
//      called by rtl_run at 100hz or more
void Copter::dynamic_rtl_climb_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }

    // check if we've completed this stage of RTL 
    rtl_state_complete = wp_nav->reached_wp_destination();
}

void Copter::dynamic_rtl_return_start()
{
    rtl_state = RTL_ReturnHome;
    rtl_state_complete = false;

    // Uses guided mode's velocity controller for the return phase
    guided_init(true);
}

// rtl_return_run - implements the fly to target portion of DRTL using the guided velocity controller
//      called by rtl_run at 100hz or more
void Copter::dynamic_rtl_return_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }


    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to target
    Vector3f vel_of_target;  // velocity of lead vehicle
     
    static uint16_t loiter_counter = 0;

    if (dynamic_rtl_get_target_dist_and_vel(dist_vec, vel_of_target)) {
         // convert dist_vec to cm in NEU
        const Vector3f dist_vec_neu(dist_vec.x * 100.0f, dist_vec.y * 100.0f, 0.0f);

        // Calculate Desired velocity with P controller. Maintain level flight during return phase
        desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_neu.x * g2.drtl_kp);
        desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_neu.y * g2.drtl_kp);
        desired_velocity_neu_cms.z = 0;

         // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = norm(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_speed_xy())) {
            const float scalar_xy = pos_control->get_speed_xy() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_speed_xy();
        }
        
        guided_set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false);
        // Run Guided's Velocity controller
        guided_run();
        
        
        if (norm(dist_vec_neu.x, dist_vec_neu.y) < g2.drtl_radius_cm) {
            loiter_counter++;
        } else {
            loiter_counter = 0;
        }
        

    } else {
        // Figure out how to handle this more gracefully

        // abort if we don't have an updated position from the target
        if (!set_mode(LOITER,  MODE_REASON_INVALID_TARGET)) {
            set_mode(ALT_HOLD,  MODE_REASON_INVALID_TARGET);
        }
        gcs_send_text(MAV_SEVERITY_INFO, "DRTL abort: target too far");
        return;
    }
    
    // check if we've completed this stage of RTL
    if (((loiter_counter * MAIN_LOOP_MICROS)/1000.0f) >= (uint32_t)g.rtl_loiter_time.get()) {
        // we have loitered long enough
        rtl_state_complete = true;
        loiter_counter = 0;
    }
}

// rtl_descent_start - initialise descent to final alt
void Copter::dynamic_rtl_descent_start()
{
    rtl_state = RTL_FinalDescent;
    rtl_state_complete = false;
        
    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
    
    float descent_speed_down = wp_nav->get_speed_down();
    
    if (g.land_speed_high > 0) {
        descent_speed_down = g.land_speed_high;
    }
    pos_control->set_speed_z(-descent_speed_down, wp_nav->get_speed_up());
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void Copter::dynamic_rtl_descent_run()
{

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // set target to current position
        wp_nav->init_loiter_target();
        return;
    }

    // process pilot's input
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);


    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to target
    Vector3f vel_of_target;  // velocity of lead vehicle
    
    static uint16_t preland_counter = 0;
    
    if (dynamic_rtl_get_target_dist_and_vel(dist_vec, vel_of_target)) {
         // convert dist_vec to cm in NEU, Ignore altitude
        const Vector3f dist_vec_neu(dist_vec.x * 100.0f, dist_vec.y * 100.0f, -dist_vec.z * 100.0f + (float)g2.drtl_alt_land_cm);

        // Calculate Desired velocity with P controller. 
        desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_neu.x * g2.drtl_kp);
        desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_neu.y * g2.drtl_kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_neu.z * g2.drtl_kp * 0.5f); 
        
        // constrain downward velocity to descent speed
        if (desired_velocity_neu_cms.z < pos_control->get_speed_down()) {
            desired_velocity_neu_cms.z = pos_control->get_speed_down();
        }
        

         // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = norm(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_speed_xy())) {
            const float scalar_xy = pos_control->get_speed_xy() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_speed_xy();
        }
        guided_set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false);
        guided_run();
        
        // Complete when we are within 10 cm (z) and 1m (xy) of the descent target. 
        if ((fabs(dist_vec_neu.z) < 10.0f) && (norm(dist_vec_neu.x, dist_vec_neu.y) < 100.0f)) {
            preland_counter++;
        } else {
            preland_counter = 0;
        }
        
        // wait 10 loops
        rtl_state_complete = (preland_counter > 10); 
        
    } else {
        // abort if we don't have an updated position from the target
        if (!set_mode(LOITER,  MODE_REASON_INVALID_TARGET)) {
            set_mode(ALT_HOLD,  MODE_REASON_INVALID_TARGET);
        }
        gcs_send_text(MAV_SEVERITY_INFO, "DRTL abort: target invalid");
    }
}

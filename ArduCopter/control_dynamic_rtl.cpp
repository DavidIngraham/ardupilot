#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

/*
 * Init and run calls for dynamic RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

#define AP_GCS_INTERVAL_MS 1000 // Debug prints to GCS interval
#define DYNAMIC_RTL_TIMEOUT_MS 3000 // Timeout value to mark target position as invalid

// rtl_init - initialise rtl controller
bool Copter::dynamic_rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        dynamic_rtl_build_path(!failsafe.terrain);
        rtl_climb_start();
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
            rtl_return_start();
            break;
        case RTL_ReturnHome:
            rtl_loiterathome_start();
            break;
        case RTL_LoiterAtHome:
            if (rtl_path.land || failsafe.radio) {
                rtl_land_start();
            }else{
                rtl_descent_start();
            }
            break;
        case RTL_FinalDescent:
            // do nothing
            break;
        case RTL_Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (rtl_state) {

    case RTL_InitialClimb:
        rtl_climb_return_run();
        break;

    case RTL_ReturnHome:
        rtl_climb_return_run();
        break;

    case RTL_LoiterAtHome:
        rtl_loiterathome_run();
        break;

    case RTL_FinalDescent:
        rtl_descent_run();
        break;

    case RTL_Land:
        rtl_land_run();
        break;
    }
}

// handle mavlink MSG_ID_GLOBAL_POSITION_INT messages
void Copter::dynamic_rtl_handle_msg(mavlink_message_t *msg)
{
    gcs_send_text(MAV_SEVERITY_INFO,"Updating DRTL Position");
    // exit immediatley if we're not in DYNAMIC_RTL mode
    //if (control_mode != DYNAMIC_RTL) {
    	//return; 
    //} Just kidding - We want to log this all the time for now

    // skip our own messages
    if (msg->sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if (msg->sysid != g2.drtl_sysid_to_target) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO,"DRTL SYSID Invalid %u %u", (unsigned)msg->sysid, (unsigned)g2.drtl_sysid_to_target);
        //return;
    }
    gcs_send_text(MAV_SEVERITY_INFO,"DRTL SYSID Valid");

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
        drtl_target_location.alt = packet.alt / 10;     // convert millimeters to cm
        drtl_target_velocity_ned.x = packet.vx * 0.01f; // velocity north
        drtl_target_velocity_ned.y = packet.vy * 0.01f; // velocity east
        drtl_target_velocity_ned.z = packet.vz * 0.01f; // velocity down
        drtl_last_location_update_ms = now;
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            drtl_target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
            drtl_last_heading_update_ms = now;
        }



        if ((AP_HAL::millis() - drtl_last_location_sent_to_gcs > AP_GCS_INTERVAL_MS)) {
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "DRTL: %u %ld %ld %4.2f\n",
                            (unsigned)msg->sysid,
                            (long)drtl_target_location.lat,
                            (long)drtl_target_location.lng,
                            (double)(drtl_target_location.alt * 0.01f));    // cm to m
            drtl_last_location_sent_to_gcs = now;
        }
    }
}

// get target's estimated location
bool Copter::dynamic_rtl_get_target_location_and_velocity(Location &loc, Vector3f &vel_ned)
{

    // check for timeout
    if ((drtl_last_location_update_ms == 0) || ((AP_HAL::millis() - drtl_last_location_update_ms) > DYNAMIC_RTL_TIMEOUT_MS)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "drtl timestamps %u %u \n", (unsigned)drtl_last_location_update_ms, (unsigned)AP_HAL::millis());
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - drtl_last_location_update_ms) * 0.001f;

    // project the vehicle position
    Location last_loc = drtl_target_location;
    location_offset(last_loc, vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 10.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    return true;
}


void Copter::dynamic_rtl_build_path(bool terrain_following_allowed)
{
    // origin point is our stopping point
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->get_stopping_point_z(stopping_point);
    rtl_path.origin_point = Location_Class(stopping_point);
    rtl_path.origin_point.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME);

    // compute return target
    dynamic_rtl_compute_return_target(terrain_following_allowed);

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location_Class(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location_Class(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location_Class::ALT_FRAME_ABOVE_HOME);

    // set land flag
    rtl_path.land = g.rtl_alt_final <= 0;
}

// compute the return target - home or rally point
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
		set_mode(LOITER, MODE_REASON_INVALID_TARGET);
        gcs_send_text(MAV_SEVERITY_INFO, "DRTL abort: target invalid");
		return;
	}

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = current_loc.alt;

    // decide if we should use terrain altitudes
    rtl_path.terrain_used = terrain_use() && terrain_following_allowed;
    if (rtl_path.terrain_used) {
        // attempt to retrieve terrain alt for current location, stopping point and origin
        int32_t origin_terr_alt, return_target_terr_alt;
        if (!rtl_path.origin_point.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, origin_terr_alt) ||
            !rtl_path.return_target.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, return_target_terr_alt) ||
            !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_alt)) {
            rtl_path.terrain_used = false;
            Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        }
    }

    // convert return-target alt (which is an absolute alt) to alt-above-home or alt-above-terrain
    if (!rtl_path.terrain_used || !rtl_path.return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_TERRAIN)) {
        if (!rtl_path.return_target.change_alt_frame(Location_Class::ALT_FRAME_ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_cm(0, Location_Class::ALT_FRAME_ABOVE_HOME);
        }
        rtl_path.terrain_used = false;
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon use_terrain
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    target_alt = MAX(target_alt, curr_alt + MAX(0, g.rtl_climb_min));
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

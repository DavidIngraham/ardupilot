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
        rtl_build_path(!failsafe.terrain);
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

    // exit immediatley if we're not in DYNAMIC_RTL mode
    if (control_mode != DYNAMIC_RTL) {
    	//return;
    }

    // skip our own messages
    if (msg->sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if (msg->sysid != g2.drtl_sysid_to_target) {
        return;
    }

    // decode global-position-int message
    if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        const uint32_t now = AP_HAL::millis();

        // get estimated location and velocity (for logging)
        Location loc_estimate{};
        Vector3f vel_estimate;
        UNUSED_RESULT(dynamic_rtl_get_target_location_and_velocity(loc_estimate, vel_estimate));

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
                            (unsigned)g2.drtl_sysid_to_target,
                            (long)drtl_target_location.lat,
                            (long)drtl_target_location.lng,
                            (double)(drtl_target_location.alt * 0.01f));    // cm to m
        }

        // log lead's estimated vs reported position
        DataFlash_Class::instance()->Log_Write("DRTL",
                                               "TimeUS,Lat,Lon,Alt,VelX,VelY,VelZ,LatE,LonE,AltE",  // labels
                                               "sDUmnnnDUm",    // units
                                               "F--B000--B",    // mults
                                               "QLLifffLLi",    // fmt
                                               AP_HAL::micros64(),
                                               drtl_target_location.lat,
                                               drtl_target_location.lng,
                                               drtl_target_location.alt,
                                               (double)drtl_target_velocity_ned.x,
                                               (double)drtl_target_velocity_ned.y,
                                               (double)drtl_target_velocity_ned.z,
                                               loc_estimate.lat,
                                               loc_estimate.lng,
                                               loc_estimate.alt
                                               );
    }
}

// get target's estimated location
bool Copter::dynamic_rtl_get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const
{

    // check for timeout
    if ((drtl_last_location_update_ms == 0) || (AP_HAL::millis() - drtl_last_location_update_ms > DYNAMIC_RTL_TIMEOUT_MS)) {
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
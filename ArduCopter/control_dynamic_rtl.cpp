#include "Copter.h"

/*
 * Init and run calls for dyanmic RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */


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
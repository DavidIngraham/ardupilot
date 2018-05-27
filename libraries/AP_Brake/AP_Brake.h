#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <DataFlash/DataFlash.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>

// Brake accel control default gains
#define AP_BRAKE_RATE_P     1.00f
#define AP_BRAKE_RATE_I     0.50f
#define AP_BRAKE_RATE_IMAX  1.00f
#define AP_BRAKE_RATE_D     0.00f
#define AP_BRAKE_RATE_FILT  5.00f
#define AP_BRAKE_RATE_DT    0.10f
#define AP_BRAKE_ACC_MAX    GRAVITY_MSS * 0.5f

class AP_Brake {
public:

	AP_Brake();

	// indicate whether this module is enabled
    bool enabled() const { return _enabled; }

    // initialise the brake
    void init(float loop_period_s);

    // update the brake controller based on the measured acceleration
    void update(float measured_acceleration);

    // control whether the brake controller is active
    void set_active(bool active);

    // set a commanded decceleration rate for the brake
    void set_desired_deccel(float accel_command);

    // Set the braking force to trim (For parking brake)
    void set_locked(bool locked);

    // Set the deccel command to max
    void set_command_to_max();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

	// parameters
	AP_Int8     _enabled;               
	AC_PID 		_accel_pid; // accel control PID
    AP_Float    _accel_max; // Maximum accel to be commanded


	bool _active = false; //controls whether to run the PID controller. 
    bool _locked = false ; //controls whether the parking break should be on
	float _accel_command = 0; //current acceleration command (m/s/s, -x is positive)
};
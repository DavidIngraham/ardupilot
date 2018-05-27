/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Brake.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Brake::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Brake enable/disable
    // @Description: Brake enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Brake, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _ACC_P
    // @DisplayName: Brake control rate P gain
    // @Description: Brake control rate P gain.  Converts rate error (in radians/sec) to pwm output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _ACC_I
    // @DisplayName: Brake control I gain
    // @Description: Brake control I gain.  Corrects long term error between the desired rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _ACC_IMAX
    // @DisplayName: Brake control I gain maximum
    // @Description: Brake control I gain maximum.  Constrains the output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @User: Standard

    // @Param: _ACC_D
    // @DisplayName: Brake control D gain
    // @Description: Brake control D gain.  Compensates for short-term change in desired rate vs actual
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACC_FILT
    // @DisplayName: Brake control filter frequency
    // @Description: Brake control input filter.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_accel_pid, "_ACC_", 2, AP_Brake, AC_PID),

    // @Param: ACC_MAX
    // @DisplayName: Maximum Braking Acceleration Command
    // @Description: Limits the braking acceleration setpoint. Reduce if brakes are consistently skidding
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO("_ACC_MAX", 3, AP_Brake, _accel_max, AP_BRAKE_ACC_MAX),



    AP_GROUPEND
};

AP_Brake::AP_Brake():
    _accel_pid(AP_BRAKE_RATE_P, AP_BRAKE_RATE_I, AP_BRAKE_RATE_D, AP_BRAKE_RATE_IMAX, AP_BRAKE_RATE_FILT, AP_BRAKE_RATE_DT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Brake::init(float loop_period_s) 
{
    // return immediately if not enabled
    if (!_enabled.get()) {
        return;
    }

    _accel_pid.set_dt(loop_period_s);
}

// Update the Brake controller. Should be called at the main loop rate.    
void AP_Brake::update(float measured_accel) 
{
    float brake_left_percent = 0.0f;
    float brake_right_percent = 0.0f;

    if ( _locked ) {
        // Set to trim value for parking brake
        SRV_Channels::set_output_to_trim(SRV_Channel::k_brake_left);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_brake_right);
        return;

    } else if ( _active ) {
        // Run PID Controller
        float accel_error = _accel_command - measured_accel;

        float normalized_accel_error = accel_error / _accel_max; 

        _accel_pid.set_input_filter_all(normalized_accel_error);

        float output = _accel_pid.get_pid() + _accel_pid.get_ff(_accel_command / _accel_max);

        output = constrain_float(output, 0.0f, 1.0f);

        brake_left_percent = output * 100.0f;
        brake_right_percent = output * 100.0f;

    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_brake_left, brake_left_percent);
    SRV_Channels::set_output_scaled(SRV_Channel::k_brake_left, brake_right_percent);
}


// Controls whether to run the PID controller. 
// Do not run the controller when the wheels aren't on the ground
// Will not activate if _locked is set
void AP_Brake::set_active(bool active)
{
    if ( _locked ) {
        return;
    }

    _active = active;

    if (active) {
        _accel_pid.reset_I(); // reset integrator before re-activing controller
    }
    
}

// Set the desired deccel (slowing down is positive)
void AP_Brake::set_desired_deccel(float accel_command)
{
    _accel_command = constrain_float(accel_command, 0.0f, _accel_max);
}

// Set the deccel command to max
void AP_Brake::set_command_to_max()
{
    _accel_command = _accel_max;
}

// set the parking brake
void AP_Brake::set_locked(bool locked)
{
    _locked = locked;
}
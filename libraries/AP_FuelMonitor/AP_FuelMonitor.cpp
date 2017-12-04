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
/*
 *   AP_FuelMonitor.cpp - Fuel tank level sensor driver and estimator
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#include "AP_FuelMonitor.h"
#include "AP_FuelMonitor_Analog_Level.h"

extern const AP_HAL::HAL &hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define FUELSENSOR_DEFAULT_PIN 1
#else
 #define FUELSENSOR_DEFAULT_PIN 15
#endif

// Rate that the read() function is called. Used in complimentary filter logic. Generally should be called at 10Hz
#ifndef FUELSENSOR_UPDATE_DT
    #define FUELSENSOR_UPDATE_DT 0.1f
#endif


// table of user settable parameters
const AP_Param::GroupInfo AP_FuelMonitor::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Fuel monitor type
    // @Description: Type of fuel monitor
    // @Values: 0:None, 1:Sensorless, 2:Analog-Level
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_FuelMonitor, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: OFFSET
    // @DisplayName: Fuel sensor zero offset for analog level sesnors
    // @Description: Fuel sensor calibration zero offset. Set so that 0 percent is reported when the tank is considered empty
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OFFSET", 2, AP_FuelMonitor, _offset, 0),

    // @Param: RATIO
    // @DisplayName: Fuel Sensor ratio for analog level sensors
    // @Description: Fuel Sensor calibration ratio in percent/volt. Set the zero offset first, then set this so that the sensor reports 100% when the tank is considered full.
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("RATIO",  3, AP_FuelMonitor, _ratio, 20.0f),

    // @Param: PIN
    // @DisplayName: Fuel Sensor pin
    // @Description: The pin number that the fuel sensor is connected to for analog sensors. Set to 15 on the Pixhawk for the analog airspeed port. 
    // @User: Advanced
    AP_GROUPINFO("PIN",  4, AP_FuelMonitor, _pin, FUELSENSOR_DEFAULT_PIN),

    // @Param: BR_IDLE
    // @DisplayName: Fuel burn rate (ml/min)
    // @Description: The rate of fuel consumption at idle RPM. Used for sensorless mode only.
    // @User: Advanced
    AP_GROUPINFO("BR_IDLE",  5, AP_FuelMonitor, _burn_rate_idle, 0), 

    // @Param: BR_CRUISE
    // @DisplayName: Fuel burn rate (ml/min)
    // @Description: The rate of fuel consumption at cruise RPM. Used for sensorless mode only. 
    // @User: Advanced
    AP_GROUPINFO("BR_CRUISE",  6, AP_FuelMonitor, _burn_rate_cruise, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Maximum Pitch/Roll angle for valid level sensors data (degrees)
    // @Description: If this is set and the burn rates are non-zero, the fuel complimentary filter reject the level sensor and rely wholly on the burn rate estimator. This is useful for preventing the fuel tank estimate from becoming innacurate while loitering on a plane.
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX",  7, AP_FuelMonitor, _angle_max_deg, 7),  

    // @Param: VOL_MAX
    // @DisplayName: Fuel Tank Volumetric Capacity (ml)
    // @Description: Thevolume of fuel in the tank when the sensor measures 100%
    AP_GROUPINFO("VOL_MAX",  8, AP_FuelMonitor, _volume_max, 0),

    // @Param: VOL_INIT
    // @DisplayName: Volume of fuel loaded into the vehicle initially
    // @Description: This is the value of fuel assumed to be loaded into the vehicle at power on. If this is changed after boot up and a level sensor is not present the current fuel estimate will be changed to match this value.
    // @User: Advanced
    AP_GROUPINFO("VOL_INIT",  9, AP_FuelMonitor, _volume_initial, 0),      

    AP_GROUPEND
};


AP_FuelMonitor::AP_FuelMonitor()
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_FuelMonitor::init()
{

    switch ((enum fuel_sensor_type)_type.get()) {
    case TYPE_NONE:
    case TYPE_SENSORLESS:
        // nothing to do
        break;

    case TYPE_ANALOG_LEVEL:
        sensor = new AP_FuelMonitor_Analog_Level(*this);
        break;
    }

    //Initialize the sensors and set the initial value
    if (enabled()) {

        // Initally reset to the user-declared inital volume.
        _volume_remaining = _volume_initial;
        // Remember what the mass_inital parameter was set to so we can detect if it has changed.
        _previous_volume_initial = _volume_initial; 

        if (sensor && !sensor->init()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fuel sensor init failed");
            delete sensor;
            sensor = nullptr;  

        } else if (level_sensor_present()) {
            // If we have a level sensor, use it to set the initial value
            sensor->get_volume_measurement(_volume_remaining);

        } 

        _percent = volume_to_percent(_volume_remaining);
    }
        

}


// update the fuel level estimate. Should be called at ten hz. 
void AP_FuelMonitor::read(const vehicle_status_t vehicle_status)
{
    if (!enabled()) {
        return;
    }

    // Detect if the user has changed the initial volume parameter. When it is changed, use this user-set value as the current volume remaining
    if ( fabs(_volume_initial - _previous_volume_initial) > 0.1f) {
        _volume_remaining = _volume_initial;
        _previous_volume_initial = _volume_initial;
    }

    float fuel_burn_rate = 0;

    // Calculate the fuel burn rate
    if (vehicle_status.engine_armed) {
        // This uses a simple 2 point linear model which is probably not very accurate away from cruise speed. 
        // We only use this if the relevant parameters are all nonzero
        if (vehicle_status.throttle_percent_cruise > 0 && _burn_rate_idle > 0 && _burn_rate_cruise > 0)
        {
            fuel_burn_rate = (vehicle_status.throttle_percent / vehicle_status.throttle_percent_cruise) * (_burn_rate_cruise - _burn_rate_idle) + _burn_rate_idle;
        }
    }

    // Subtract estimated fuel burn from total volume estimate
    _volume_remaining -= fuel_burn_rate * FUELSENSOR_UPDATE_DT; 

    // Use a complimentary filter to fuse in level sensor data
    if (level_sensor_present() &&
        fabs(vehicle_status.pitch_deg) <= _angle_max_deg &&
        fabs(vehicle_status.roll_deg) <= _angle_max_deg) {

        float sensor_measurement = 0;

        sensor->get_volume_measurement(sensor_measurement);

        _volume_remaining = (0.99f * _volume_remaining) + (0.01 * sensor_measurement);

    }

    _percent = volume_to_percent(_volume_remaining);

}


float AP_FuelMonitor::percent_to_volume(int16_t percent)
{
    return percent * 0.01f * _volume_max; 
}

int16_t AP_FuelMonitor::volume_to_percent(float volume)
{
    if (_volume_max <= 0) { return 0; }

    return round(_volume_remaining / _volume_max * 100.0f);
}

// External interface to set fuel level via GCS
void AP_FuelMonitor::set_level_percent(int16_t percent) 
{
    _percent = percent;
}

// External interface to set fuel level via GCS
void AP_FuelMonitor::set_level_volume(float volume)
{
    _volume_remaining = volume;
}


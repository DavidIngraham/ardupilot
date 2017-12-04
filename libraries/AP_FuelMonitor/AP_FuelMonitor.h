#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "AP_FuelMonitor_Backend.h"


class AP_FuelMonitor
{
public:
    friend class AP_FuelMonitor_Backend; 
    
    // constructor
    AP_FuelMonitor();

    static AP_FuelMonitor create() { return AP_FuelMonitor{}; }

    constexpr AP_FuelMonitor(AP_FuelMonitor &&other) = default;

    /* Do not allow copies */
    AP_FuelMonitor(const AP_FuelMonitor &other) = delete;
    AP_FuelMonitor &operator=(const AP_FuelMonitor&) = delete;

    //Vehicle Status struct used to get updates from the vehicle code on every read()
    struct vehicle_status_t {
        int8_t throttle_percent;
        int8_t throttle_percent_cruise;
        float roll_deg;
        float pitch_deg;
        bool engine_armed;
    };
    

    void init(void);

    void read(const vehicle_status_t vehicle_status);

    // Used to update level from GCS
    void set_level_percent(int16_t percent);
    void set_level_volume(float volume);

    float get_percent(void) const {
        return _percent;
    }

    float get_volume_remaining(void) const {
        return _volume_remaining;
    }

    float get_burn_rate(void) const {
        return _burn_rate;
    }

        // return true if fuel monitor is enabled
    bool enabled(void) const {
        return _type.get() != TYPE_NONE;
    }

        // return true if we have a working level sensor
    bool level_sensor_present(void) const {
        return sensor && (_type.get() == TYPE_ANALOG_LEVEL);
    }


    static const struct AP_Param::GroupInfo var_info[];

    enum fuel_sensor_type {
        TYPE_NONE=0,
        TYPE_SENSORLESS=1,
        TYPE_ANALOG_LEVEL=2,
    };

private:

    AP_Float        _offset;
    AP_Float        _ratio;
    AP_Float        _volume_initial;
    AP_Float        _volume_max;
    AP_Float        _burn_rate_idle;
    AP_Float        _burn_rate_cruise;
    AP_Int8         _angle_max_deg;
    AP_Int8         _type;
    AP_Int8         _pin;

    float        _previous_volume_initial;
    int16_t      _percent;
    float        _volume_remaining;
    float        _burn_rate;

    bool		    _healthy:1;
    uint32_t        _last_update_ms;

    float percent_to_volume(int16_t percent);
    int16_t volume_to_percent(float volume);

    AP_FuelMonitor_Backend *sensor;
};

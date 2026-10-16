#pragma once

#include <AP_TECS/AP_TECS_config.h>

#if AP_TECS_PARAGLIDER_ENABLED
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>

// Differential-brake control of heading rate, independent of the roll PID.
class PG_TurnController {
public:
    PG_TurnController();
    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable.get() != 0; }
    bool active() const { return _initialised; }
    void reset();
    // Reset for return from an output override, matching its last scaled demand.
    void reset(float output_scaled);
    static float heading_rate_target(float bank_rad, float cos_pitch, float airspeed, float course_projection);
    static Vector2f allocate_brakes(float differential_pct, float common_pct);
    float update(float target_dps, float measured_dps, float roll_rate_dps, float airspeed, float dt, float brake_limit = 1.0f);

private:
    AP_Int8 _enable;
    AP_Float _ff;
    AP_Float _p;
    AP_Float _i;
    AP_Float _imax;
    AP_Float _rate_max;
    AP_Float _accel_max;
    AP_Float _filter_hz;
    AP_Float _tconst;
    AP_Float _reference_speed;
    AP_Float _roll_damp;
    AP_Float _dff;

    LowPassFilterFloat _rate_filter;
    LowPassFilterFloat _roll_rate_filter;
    float _target = 0;
    float _integrator = 0;
    bool _initialised = false;
    bool _match_output = false;
    float _handover_output = 0;
    float _handover_offset = 0;
};
#endif // AP_TECS_PARAGLIDER_ENABLED

#include "AP_TECS_config.h"

#if AP_TECS_PARAGLIDER_ENABLED

#include "AP_TECS.h"
#include <AP_Landing/AP_Landing.h>

const AP_Param::GroupInfo AP_TECS::Paraglider_Params::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Paraglider TECS enable
    // @Description: Enable paraglider-specific throttle control (TECS_PG_ENABLE)
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_TECS::Paraglider_Params, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PR_FILT
    // @DisplayName: Paraglider pitch-rate damper filter frequency
    // @Description: Low-pass filter cutoff frequency for pitch-rate signal used in damper (Hz). Set to 0 to disable filtering.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("PR_FILT", 1, AP_TECS::Paraglider_Params, pr_filt_hz, 2.0f),

    AP_GROUPEND

};


void AP_TECS::_update_pitch_rate(void) 
{
    // Apply low-pass filter to pitch rate for paraglider if enabled
    if (_pg_params.enable && _ahrs.healthy()  && _pg_params.pr_filt_hz > 0.0f) {
        const Vector3f &gyro = _ahrs.get_gyro();
        _pg_pitch_rate_lpf.apply(gyro.y, _DT);
    } else {
        _pg_pitch_rate_lpf.reset(0.0f);
    }
}

void AP_TECS::_reset_paraglider(void) 
{
    _pg_pitch_rate_lpf.set_cutoff_frequency(_pg_params.pr_filt_hz);
    _pg_pitch_rate_lpf.reset();
    reset_throttle_I();
}


void AP_TECS::_update_paraglider_hgt_demand(void)
{

    _hgt_dem_in = _hgt_dem_in_raw;

    // 2-sample smoothing of command 
    const float hgt_in = 0.5f * (_hgt_dem_in + _hgt_dem_in_prev);
    _hgt_dem_in_prev = _hgt_dem_in;


    // Don't allow height demand to get too far ahead of the vehicles current height
    // if vehicle is unable to follow the demanded climb or descent

    // large height errors will result in the throttle saturating
    bool max_climb_condition  = (_thr_clip_status == clipStatus::MAX) &&
                                !((_flight_stage == AP_FixedWing::FlightStage::TAKEOFF) || (_flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING));
    bool max_descent_condition = (_thr_clip_status == clipStatus::MIN) && !_landing.is_flaring();
    
    const float hgt_dem_alpha = _DT / MAX(_DT + _hgt_dem_tconst, _DT);
    if (max_climb_condition && _hgt_dem > _hgt_dem_prev) {
        _max_climb_scaler *= (1.0f - hgt_dem_alpha);
    } else if (max_descent_condition && _hgt_dem < _hgt_dem_prev) {
        _max_sink_scaler *= (1.0f - hgt_dem_alpha);
    } else {
        _max_climb_scaler = _max_climb_scaler * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
        _max_sink_scaler  =  _max_sink_scaler * (1.0f - hgt_dem_alpha) + hgt_dem_alpha;
    }

    _climb_rate_limit = _maxClimbRate * _max_climb_scaler;
    _sink_rate_limit = _maxSinkRate * _max_sink_scaler;

    // Slew-limit the altitude trajectory 
    const float up_step_max = MAX(_climb_rate_limit, 0.0f) * _DT;
    const float dn_step_max = MAX(_sink_rate_limit, 0.0f) * _DT;

    float step = hgt_in - _hgt_dem_rate_ltd;
    step = constrain_float(step, -dn_step_max, up_step_max);
    _hgt_dem_rate_ltd += step;

    // Publish the setpoint and its derivative
    _hgt_dem = _hgt_dem_rate_ltd;

    _hgt_rate_dem = (_hgt_dem - _hgt_dem_prev) / _DT;
    _hgt_dem_prev = _hgt_dem;
}


/*
  Calculate throttle for a powered paraglider

  Paragliders do not have a pitch actuator. The wing flies at a nearly constant airspeed, and throttle is used to control vertical speed.
  Unlike normal TECS mode, this controller does not concern itself with kinetic energy or energy balance. 

  This uses a PI + FF controller to achieve a desired vertical speed, similarly to _update_throttle_with_airspeed. 
  Gains are calculated from standard TECS params, scaled for sane defaults.
  
  A damper is included with negative feedback from pitch-rate to throttle demand. This helps to 
 */
void AP_TECS::_update_paraglider(uint64_t now, float pitch_trim_deg, float hgt_afe)
{
    _using_airspeed_for_throttle = false;

    _update_throttle_limits();

    _initialise_states(hgt_afe);

    _update_paraglider_hgt_demand();
   
    _pitch_dem = pitch_trim_deg * M_PI / 180.0f;

    if (_flags.is_gliding) {
        _throttle_dem = 0.0f;
        reset_throttle_I();
        constrain_throttle();
        _log_TECS_state(now);
        return;
    }

    // Nominal throttle: use landing throttle if active else cruise. 
    // We intentionally do not respect throttle nudge for paragliders as it is not possible to trim for level flight at a different throttle
    const float nomThr = (_flags.is_doing_auto_land && _landThrottle >= 0) ? (_landThrottle * 0.01f) : (aparm.throttle_cruise * 0.01f);

    // Height error 
    const float hgt_err_m = _height - _hgt_dem;

    // Convert height error to vertical speed demand with a time constant.
    // This is the outer outer loop for paraglider throttle-only altitude control.
    float v_up_des = -hgt_err_m / _hgt_dem_tconst;

    // feed-forward from hgt_rate_dem (setpoint rate of change)
    v_up_des += _hgt_rate_dem;

    // Apply climb/sink limits
    v_up_des = constrain_float(v_up_des, -_maxSinkRate, _maxClimbRate);

    // Scale our nominal throttle based on the desired climb rate
    // This avoid excessive reliance on the integrator for steady state climbs
    float thr_ff = nomThr;
    if (v_up_des > 0 && _maxClimbRate > 0.1f) {
        thr_ff = nomThr + (v_up_des / _maxClimbRate) * (_THRmaxf - nomThr);
    } else if (v_up_des < 0 && _minSinkRate > 0.1f) {
        thr_ff = nomThr + (v_up_des / _maxSinkRate) * (nomThr - _THRminf);
    }

    const float v_err = v_up_des - _climb_rate;

    const float thr_min_clip0 = constrain_float(_THRminf, 0.0f, _THRmaxf);

    // Map (m/s) -> throttle using TECS-style scaling:
    // K ~ (throttle_range) / (timeConstant * accel_limit)
    const float a_lim = MAX(_vertAccLim, 0.1f);
    const float K_v2thr = (_THRmaxf - thr_min_clip0) / (timeConstant() * a_lim);

    float thr_ff_pd = thr_ff + v_err * K_v2thr;
    thr_ff_pd = constrain_float(thr_ff_pd, _THRminf, _THRmaxf);

    // Slew-limit P+FF throttle
    if (aparm.throttle_slewrate != 0) {
        const float thr_rate_incr =
            _DT * (_THRmaxf - thr_min_clip0) * aparm.throttle_slewrate * 0.01f;

        thr_ff_pd = constrain_float(
            thr_ff_pd,
            _last_throttle_dem - thr_rate_incr,
            _last_throttle_dem + thr_rate_incr
        );
        _last_throttle_dem = thr_ff_pd;
    }

    // Integrator limits + update
    const float max_amp = 0.5f * (_THRmaxf - thr_min_clip0); // 50% of positive throttle range
    const float integ_max = constrain_float((_THRmaxf - thr_ff_pd + 0.1f), -max_amp, max_amp);
    const float integ_min = constrain_float((_THRminf - thr_ff_pd - 0.1f), -max_amp, max_amp);

    _integTHR_state += (v_err * _get_i_gain()) * _DT * K_v2thr;
    _integTHR_state = constrain_float(_integTHR_state, integ_min, integ_max);

    float pitch_damp = _ptchDamp;
    if (!is_zero(_land_pitch_damp) && _flags.is_doing_auto_land) {
        pitch_damp = _land_pitch_damp;
    }
    float pr_damp = -pitch_damp * _pg_pitch_rate_lpf.get();
    pr_damp = constrain_float(pr_damp, -0.25f, 0.25f);

    _throttle_dem = thr_ff_pd + _integTHR_state + pr_damp;

    if (_landing.is_flaring()) {
        pitch_damp = _landDamp;
    }

    constrain_throttle();

    _log_TECS_state(now);

    #if 1
    static uint32_t last_ms;
    if (AP_HAL::millis() - last_ms > 1000) {
        last_ms = AP_HAL::millis();
        ::printf("V_up_des=%.2f V_up=%.2f, ff_p_out=%0.2f, int_out=%0.2f, pr_damp=%0.2f \n",
                 v_up_des,
                 _climb_rate,
                 thr_ff_pd,
                 _integTHR_state,
                 pr_damp
        );
    }
    #endif

}
    

#include "AP_TECS.h"


#endif //AP_TECS_PARAGLIDER_ENABLED
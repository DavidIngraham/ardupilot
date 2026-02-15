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

    // @Param: THR_P
    // @DisplayName: Paraglider Throttle Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    // @User: Advanced

    // @Param: THR_I
    // @DisplayName: Paraglider Throttle Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    // @User: Advanced

    // @Param: THR_IMAX
    // @DisplayName: Paragider Throttle PI Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    // @User: Advanced
    AP_SUBGROUPINFO(thr_pi, "THR_", 1, AP_TECS::Paraglider_Params, AC_PI),

    // @Param: PR_FILT
    // @DisplayName: Paraglider pitch-rate damper filter frequency
    // @Description: Low-pass filter cutoff frequency for pitch-rate signal used in damper (Hz). Set to 0 to disable filtering.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("PR_FILT", 2, AP_TECS::Paraglider_Params, pr_filt_hz, 2.0f),

    AP_GROUPEND

};


void AP_TECS::_update_pitch_rate(void) {
    // Apply low-pass filter to pitch rate for paraglider if enabled
    if (_pg_params.enable && _ahrs.healthy()  && _pg_params.pr_filt_hz > 0.0f) {
        const Vector3f &gyro = _ahrs.get_gyro();
        _pg_pitch_rate_lpf.apply(gyro.y, _DT);
    } else {
        _pg_pitch_rate_lpf.reset(0.0f);
    }
}

/*
  Calculate throttle for a powered paraglider

  Paragliders do not have a pitch actuator. The wing flies at a nearly constant airspeed, and throttle is used to control vertical speed.
  This uses a PI controller to achieve a desired vertical speed, along with a pitch_rate damper.
  Safety: requires AHRS for pitch-rate damper; if AHRS invalid, damper is disabled and PID integrator is reset.

 */
void AP_TECS::_update_paraglider(uint64_t now, float pitch_trim_deg)
{
    _pitch_dem = pitch_trim_deg * M_PI / 180.0f;
    _using_airspeed_for_throttle = false;

    if (_flags.is_gliding) {
        _throttle_dem = 0.0f;
        _pg_params.thr_pi.reset_I();
        return;
    }

    // Nominal throttle: use landing throttle if active else cruise. 
    // We intentionally do not respect throttle nudge for paragliders as it is not possible to trim for level flight at a higher throttle
    const float nomThr = (_flags.is_doing_auto_land && _landThrottle >= 0) ? (_landThrottle * 0.01f) : (aparm.throttle_cruise * 0.01f);

    // Height error (positive => we are below demand => command climb)
    const float hgt_err_m = (_hgt_dem - _height);

    // Convert height error to vertical speed demand with a time constant.
    // This is the outer outer loop for paraglider throttle-only altitude control.
    float v_up_des = hgt_err_m / _hgt_dem_tconst;

    // feed-forward (e.g., when mission altitude is changing)
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

    // Inner loop: PI controller (AC_PID) to convert vertical speed error to throttle correction
    // This uses the clip status from the last iteration
    const bool throttle_saturated_low = (_thr_clip_status == clipStatus::MIN);
    const bool throttle_saturated_high = (_thr_clip_status == clipStatus::MAX);
    const float pi_out = _pg_params.thr_pi.update(_climb_rate, v_up_des, _DT, throttle_saturated_low, throttle_saturated_high);

    // Pitch-rate damper: get pitch rate (rad/s) from AHRS and apply damping using existing PTCH_DAMP gain

    float pitch_damp = _ptchDamp;
    if (!is_zero(_land_pitch_damp) && _flags.is_doing_auto_land) {
        pitch_damp = _land_pitch_damp;
    }
    float pr_damp = -pitch_damp * _pg_pitch_rate_lpf.get();
    pr_damp = constrain_float(pr_damp, -0.25f, 0.25f);


    _throttle_dem = thr_ff + pi_out + pr_damp;

    if (_landing.is_flaring()) {
        pitch_damp = _landDamp;
    }

    constrain_throttle();

    _log_TECS_state(now);

    #if 0
    static uint32_t last_ms;
    if (AP_HAL::millis() - last_ms > 1000) {
        last_ms = AP_HAL::millis();
        ::printf("V_up_des=%.2f V_up=%.2f, pi_out=%0.2f, pr_damp=%0.2f \n",
                 v_up_des,
                 _climb_rate,
                 pi_out,
                 pr_damp
        );
    }
    #endif

}

void AP_TECS::_reset_paraglider(void) {
    _pg_pitch_rate_lpf.set_cutoff_frequency(_pg_params.pr_filt_hz);
    _pg_pitch_rate_lpf.reset();
    _pg_params.thr_pi.reset_I();
}
    

#include "AP_TECS.h"


#endif //AP_TECS_PARAGLIDER_ENABLED
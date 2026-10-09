#include "PG_TurnController.h"

#if AP_TECS_PARAGLIDER_ENABLED
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo PG_TurnController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Paraglider yaw-rate controller enable
    // @Description: Use heading-rate feedback and differential brake instead of the roll controller when paraglider brake outputs are assigned. Manual mode retains direct pilot brake control.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, PG_TurnController, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FF
    // @DisplayName: Paraglider yaw-rate feedforward
    // @Description: Differential brake fraction per degree per second of commanded heading rate at ASPD. A value of 0.05 requests half brake for 10 degrees per second.
    // @Range: 0 0.2
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("FF", 1, PG_TurnController, _ff, 0.051f),

    // @Param: P
    // @DisplayName: Paraglider yaw-rate proportional gain
    // @Description: Differential brake fraction per degree per second of heading-rate error, scaled by ASPD divided by true airspeed.
    // @Range: 0 0.2
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("P", 2, PG_TurnController, _p, 0.04f),

    // @Param: I
    // @DisplayName: Paraglider yaw-rate integral gain
    // @Description: Differential brake fraction per degree of accumulated heading-rate error. Integration is bounded by IMAX and prevented from increasing output saturation.
    // @Range: 0 0.1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("I", 3, PG_TurnController, _i, 0.002f),

    // @Param: IMAX
    // @DisplayName: Paraglider yaw-rate integrator limit
    // @Description: Maximum fraction of full differential brake contributed by the integrator.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("IMAX", 4, PG_TurnController, _imax, 0.15f),

    // @Param: RMAX
    // @DisplayName: Paraglider maximum heading rate
    // @Description: Maximum magnitude of the commanded heading rate. This limit does not guarantee that the aircraft can achieve that rate.
    // @Range: 1 90
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RMAX", 5, PG_TurnController, _rate_max, 20.0f),

    // @Param: ACCEL
    // @DisplayName: Paraglider heading-rate command slew limit
    // @Description: Maximum change per second of the heading-rate command after command smoothing.
    // @Range: 1 180
    // @Units: deg/s/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ACCEL", 6, PG_TurnController, _accel_max, 40.0f),

    // @Param: FILT
    // @DisplayName: Paraglider heading-rate feedback filter
    // @Description: Low-pass cutoff for heading-rate feedback. Zero disables filtering.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FILT", 7, PG_TurnController, _filter_hz, 4.0f),

    // @Param: TC
    // @DisplayName: Paraglider heading-rate command time constant
    // @Description: Time constant of heading-rate command smoothing. Zero disables smoothing; the ACCEL slew limit still applies.
    // @Range: 0 2
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TC", 8, PG_TurnController, _tconst, 0.1f),

    // @Param: ASPD
    // @DisplayName: Paraglider reference true airspeed
    // @Description: True airspeed at which FF and P were tuned. Their output scales inversely with true airspeed to account for brake effectiveness.
    // @Range: 2 30
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("ASPD", 9, PG_TurnController, _reference_speed, 5.0f),

    // @Param: RDAMP
    // @DisplayName: Paraglider roll-rate damping
    // @Description: Differential brake fraction per degree per second of body roll rate, opposing roll motion. This damps the coupled canopy roll/yaw mode without commanding a bank angle.
    // @Range: 0 0.1
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("RDAMP", 10, PG_TurnController, _roll_damp, 0.03f),

    // @Param: D_FF
    // @DisplayName: Paraglider heading acceleration feedforward
    // @Description: Differential brake fraction per degree per second squared of smoothed heading-rate command acceleration at ASPD. Output scales with the inverse square of true airspeed. Zero disables acceleration feedforward.
    // @Range: 0 0.05
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("D_FF", 11, PG_TurnController, _dff, 0.02f),

    AP_GROUPEND
};

PG_TurnController::PG_TurnController()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void PG_TurnController::reset()
{
    _target = 0;
    _integrator = 0;
    _initialised = false;
}

float PG_TurnController::update(float target_dps, float measured_dps, float roll_rate_dps, float airspeed, float dt)
{
    if (!isfinite(target_dps) || !isfinite(measured_dps) || !isfinite(roll_rate_dps) || !isfinite(airspeed) ||
        !is_positive(dt) || dt > 0.1f) {
        reset();
        return 0;
    }
    if (!_initialised) {
        _rate_filter.reset(measured_dps);
        _roll_rate_filter.reset(roll_rate_dps);
        _target = measured_dps;
        _initialised = true;
    }
    _rate_filter.set_cutoff_frequency(MAX(_filter_hz.get(), 0.0f));
    const float actual = _rate_filter.apply(measured_dps, dt);
    _roll_rate_filter.set_cutoff_frequency(MAX(_filter_hz.get(), 0.0f));
    const float roll_rate = _roll_rate_filter.apply(roll_rate_dps, dt);
    const float rate_limit = MAX(_rate_max.get(), 1.0f);
    const float requested = constrain_float(target_dps, -rate_limit, rate_limit);
    _target = constrain_float(_target, -rate_limit, rate_limit);
    const float previous_target = _target;
    const float alpha = dt / (dt + MAX(_tconst.get(), 0.0f));
    const float max_change = MAX(_accel_max.get(), 1.0f) * dt;
    _target += constrain_float(alpha * (requested - _target), -max_change, max_change);
    _target = constrain_float(_target, -rate_limit, rate_limit);

    const float scale = constrain_float(MAX(_reference_speed.get(), 2.0f) / MAX(airspeed, 2.0f), 0.25f, 2.5f);
    const float dff = MAX(_dff.get(), 0.0f) * (_target - previous_target) / dt * sq(scale);
    const float error = _target - actual;
    const float ff = MAX(_ff.get(), 0.0f) * _target * scale;
    const float p = MAX(_p.get(), 0.0f) * error * scale;
    const float rdamp = -MAX(_roll_damp.get(), 0.0f) * roll_rate * scale;
    if (!is_positive(_i.get())) {
        _integrator = 0;
    }
    const float imax = constrain_float(_imax.get(), 0.0f, 1.0f);
    const float increment = MAX(_i.get(), 0.0f) * error * dt;
    const float candidate_i = constrain_float(_integrator + increment, -imax, imax);
    const float candidate_output = ff + dff + p + rdamp + candidate_i;
    if ((candidate_output <= 1.0f || !is_positive(increment)) &&
        (candidate_output >= -1.0f || !is_negative(increment))) {
        _integrator = candidate_i;
    }
    const float output = constrain_float(ff + dff + p + rdamp + _integrator, -1.0f, 1.0f);
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: PGYR
    // @Description: Paraglider heading-rate controller and roll-rate damping
    // @Field: TimeUS: Time since system startup
    // @Field: Cmd: Limited requested heading rate in degrees per second
    // @Field: Tgt: Smoothed heading-rate target in degrees per second
    // @Field: Act: Filtered measured heading rate in degrees per second
    // @Field: FF: Feedforward differential brake fraction
    // @Field: P: Proportional differential brake fraction
    // @Field: I: Integral differential brake fraction
    // @Field: RD: Roll-rate damping differential brake fraction
    // @Field: DFF: Heading acceleration feedforward differential brake fraction
    // @Field: Out: Limited differential brake fraction
    // @Field: ASpd: True airspeed in metres per second
    AP::logger().WriteStreaming("PGYR", "TimeUS,Cmd,Tgt,Act,FF,P,I,RD,DFF,Out,ASpd", "Qffffffffff",
                               AP_HAL::micros64(), requested, _target, actual, ff, p, _integrator, rdamp, dff, output, airspeed);
#endif
    return output * 4500.0f;
}
#endif // AP_TECS_PARAGLIDER_ENABLED

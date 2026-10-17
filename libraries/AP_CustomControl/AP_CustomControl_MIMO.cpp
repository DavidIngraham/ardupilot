#include "AP_CustomControl_MIMO.h"

namespace AP_CustomControl_MIMO {

bool allocate(const Demand &request, const Demand &previous,
              const Authority &a, float dt, Allocation &result)
{
    if (!request.finite() || !previous.finite() || !isfinite(dt) || dt <= 0 ||
        !isfinite(a.common_brake_max) || !isfinite(a.differential_brake_max) ||
        !isfinite(a.throttle_min) || !isfinite(a.throttle_max) ||
        !isfinite(a.brake_slew_per_s) || !isfinite(a.throttle_slew_per_s) ||
        a.common_brake_max < 0 || a.common_brake_max > 1 ||
        a.differential_brake_max < 0 || a.differential_brake_max > 1 ||
        a.throttle_min < 0 || a.throttle_max > 1 || a.throttle_min > a.throttle_max ||
        a.brake_slew_per_s < 0 || a.throttle_slew_per_s < 0) {
        return false;
    }
    result.requested = request;
    result.limits = 0;
    const float common = constrain_float(request.common_brake, 0, a.common_brake_max);
    const float differential = constrain_float(request.differential_brake,
                                               -a.differential_brake_max, a.differential_brake_max);
    float left = constrain_float(common + MAX(-differential, 0), 0, 1);
    float right = constrain_float(common + MAX(differential, 0), 0, 1);
    if (!is_equal(common, request.common_brake) || !is_equal(right-left, request.differential_brake)) {
        result.limits |= BRAKE_TRAVEL;
    }
    const float step = a.brake_slew_per_s * dt;
    const float previous_left = previous.common_brake + MAX(-previous.differential_brake, 0);
    const float previous_right = previous.common_brake + MAX(previous.differential_brake, 0);
    const float slewed_left = constrain_float(left, previous_left-step, previous_left+step);
    const float slewed_right = constrain_float(right, previous_right-step, previous_right+step);
    if (!is_equal(left, slewed_left) || !is_equal(right, slewed_right)) {
        result.limits |= BRAKE_SLEW;
    }
    // A newly reduced authority takes priority over slew matching.
    result.applied.common_brake = constrain_float(MIN(slewed_left, slewed_right), 0, a.common_brake_max);
    const float differential_limit = MIN(a.differential_brake_max, 1-result.applied.common_brake);
    result.applied.differential_brake = constrain_float(slewed_right-slewed_left, -differential_limit, differential_limit);
    const float throttle = constrain_float(request.throttle, a.throttle_min, a.throttle_max);
    if (!is_equal(throttle, request.throttle)) {
        result.limits |= THROTTLE_TRAVEL;
    }
    const float throttle_step = a.throttle_slew_per_s * dt;
    const float slewed_throttle = constrain_float(throttle, previous.throttle-throttle_step, previous.throttle+throttle_step);
    if (!is_equal(slewed_throttle, throttle)) {
        result.limits |= THROTTLE_SLEW;
    }
    result.applied.throttle = constrain_float(slewed_throttle, a.throttle_min, a.throttle_max);
    return true;
}

} // namespace AP_CustomControl_MIMO

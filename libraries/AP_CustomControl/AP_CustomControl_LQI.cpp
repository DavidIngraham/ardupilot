#include "AP_CustomControl_LQI.h"

#if AP_PLANE_CUSTOMCONTROL_LQI_ENABLED
#include "AP_CustomControl_LQI_gains.h"
using namespace AP_CustomControl_MIMO;
using namespace MicroAGU_LQI;

bool AP_CustomControl_LQI::valid(const Input &in) const
{
    const auto &s = in.state;
    return in.eligible && in.reference.valid && s.attitude_valid && s.navigation_valid &&
        s.airspeed_valid && s.wind_valid && in.time_ms-s.time_ms <= 100 &&
        in.time_ms-in.reference.time_ms <= 500 && isfinite(in.dt) && in.dt > 0 && in.dt <= 0.1f &&
        isfinite(s.attitude_body_to_ned.length_squared()) &&
        fabsf(s.attitude_body_to_ned.length_squared()-1) < 0.01f &&
        !s.velocity_ned_mps.is_nan() && !s.velocity_ned_mps.is_inf() &&
        !s.body_rate_rads.is_nan() && !s.body_rate_rads.is_inf() &&
        !s.wind_ned_mps.is_nan() && !s.wind_ned_mps.is_inf() &&
        isfinite(s.height_home_m) && isfinite(s.true_airspeed_mps) &&
        isfinite(s.eas_to_tas) && s.eas_to_tas > 0 &&
        isfinite(in.reference.climb_rate_mps) &&
        isfinite(in.reference.height_home_m) && isfinite(in.reference.heading_rate_rads) &&
        isfinite(in.reference.equivalent_airspeed_mps);
}

bool AP_CustomControl_LQI::can_run()
{
    return valid(_frontend.mimo_input);
}

void AP_CustomControl_LQI::measurements(const Input &in, float (&y)[9]) const
{
    float roll, pitch, yaw;
    in.state.attitude_body_to_ned.to_euler(roll, pitch, yaw);
    Matrix3f rotation;
    in.state.attitude_body_to_ned.rotation_matrix(rotation);
    const Vector3f air_body = rotation.transposed() * (in.state.velocity_ned_mps-in.state.wind_ned_mps);
    y[0] = (roll-EULER[0])/XS[0];
    y[1] = (pitch-EULER[1])/XS[1];
    for (uint8_t j = 0; j < 3; j++) {
        y[2+j] = (air_body[j]-VELOCITY[j])/XS[5+j];
        y[5+j] = (in.state.body_rate_rads[j]-RATES[j])/XS[8+j];
    }
    y[8] = (in.state.height_home_m-_height_origin)/XS[15];
}

bool AP_CustomControl_LQI::engage(const Input &in)
{
    if (!valid(in) || !in.previous.applied.finite()) {
        return false;
    }
    memset(_state, 0, sizeof(_state));
    memset(_integral, 0, sizeof(_integral));
    _height_origin = in.state.height_home_m;
    _height_reference = 0;
    _turn_reference = 0;
    _elapsed = 0;
    _reset_sequence = in.state.reset_sequence;
    float y[9];
    measurements(in, y);
    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};
    for (uint8_t j = 0; j < 9; j++) {
        _state[indices[j]] = y[j];
    }
    _held = in.previous.applied;
    _applied[0] = _held.throttle;
    _applied[1] = _held.common_brake;
    _applied[2] = _held.differential_brake;
    for (uint8_t j = 0; j < 3; j++) {
        float feedback = 0;
        for (uint8_t k = 0; k < 16; k++) {
            feedback += K[j][k]*_state[k];
        }
        _matching[j] = _applied[j]-TRIM[j]+US[j]*feedback;
        _requested[j] = _applied[j];
    }
    return true;
}

bool AP_CustomControl_LQI::update_mimo(const Input &in, Demand &demand)
{
    if (!valid(in) || in.state.reset_sequence != _reset_sequence) {
        return false;
    }
    _elapsed += in.dt;
    if (_elapsed < DT) {
        demand = _held;
        return true;
    }
    // Fixed-period matrices: do not integrate a backlog after a scheduling stall.
    if (_elapsed > 2*DT) {
        return false;
    }
    _elapsed -= DT;
    float predicted[16]{};
    for (uint8_t i = 0; i < 16; i++) {
        for (uint8_t j = 0; j < 16; j++) {
            predicted[i] += A[i][j]*_state[j];
        }
        for (uint8_t j = 0; j < 3; j++) {
            predicted[i] += B[i][j]*(_applied[j]-TRIM[j])/US[j];
        }
    }
    float y[9];
    measurements(in, y);
    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};
    for (uint8_t i = 0; i < 16; i++) {
        _state[i] = predicted[i];
        for (uint8_t j = 0; j < 9; j++) {
            _state[i] += L[i][j]*(y[j]-predicted[indices[j]]);
        }
        if (!isfinite(_state[i])) {
            return false;
        }
    }
    const float climb = in.reference.vertical == VerticalReference::CLIMB_RATE ?
        in.reference.climb_rate_mps :
        (in.reference.height_home_m-_height_origin-_height_reference)*0.5f;
    const float governed_climb = constrain_float(climb, -0.5f, 0.5f);
    _height_reference += DT*governed_climb;
    const float turn = constrain_float(in.reference.heading_rate_rads, -radians(10), radians(10));
    _turn_reference += constrain_float(turn-_turn_reference, -radians(40)*DT, radians(40)*DT);
    float roll, pitch, yaw;
    in.state.attitude_body_to_ned.to_euler(roll, pitch, yaw);
    const float heading_rate = (in.state.body_rate_rads.y*sinf(roll)+in.state.body_rate_rads.z*cosf(roll))/MAX(cosf(pitch), 0.3f);
    // The fixed model has a limited speed envelope; faster demands cannot be
    // met by releasing already fully released brakes.
    const float speed_target = constrain_float(in.reference.equivalent_airspeed_mps*in.state.eas_to_tas, 3.8f, 4.4f);
    const float errors[3] = {
        _state[15]-_height_reference,
        (heading_rate-_turn_reference)/radians(10),
        (in.state.true_airspeed_mps-speed_target)/0.5f
    };
    // Map the entire allocation residual back through the integral gain matrix.
    // Saturating common brake must not freeze the independent turn integral.
    for (uint8_t j = 0; j < 3; j++) {
        float correction = 0;
        for (uint8_t i = 0; i < 3; i++) {
            correction += AW[j][i]*(_requested[i]-_applied[i]);
        }
        _integral[j] = constrain_float(_integral[j]+DT*(errors[j]+correction), -50, 50);
    }
    const float references[4] = {_height_reference, _turn_reference/radians(10),
                                (speed_target-Vector3f(VELOCITY[0], VELOCITY[1], VELOCITY[2]).length())/0.5f,
                                governed_climb};
    float target_state[16]{};
    for (uint8_t j = 0; j < 16; j++) {
        for (uint8_t r = 0; r < 4; r++) {
            target_state[j] += XREF[j][r]*references[r];
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        float feedback = 0;
        for (uint8_t j = 0; j < 16; j++) {
            feedback += K[i][j]*(_state[j]-target_state[j]);
        }
        for (uint8_t j = 0; j < 3; j++) {
            feedback += K[i][16+j]*_integral[j];
        }
        float feedforward = 0;
        for (uint8_t r = 0; r < 4; r++) {
            feedforward += UREF[i][r]*references[r];
        }
        _requested[i] = TRIM[i]+US[i]*(feedforward-feedback)+_matching[i];
        _matching[i] *= 0.5f/(0.5f+DT);
    }
    _held = {_requested[2], _requested[1], _requested[0]};
    demand = _held;
    return demand.finite();
}

void AP_CustomControl_LQI::allocation_feedback(const Allocation &allocation)
{
    _applied[0] = allocation.applied.throttle;
    _applied[1] = allocation.applied.common_brake;
    _applied[2] = allocation.applied.differential_brake;
}
#endif

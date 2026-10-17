#pragma once

#include <AP_Math/AP_Math.h>

// Vehicle/backend contract. SI units, NED navigation frame, positive-up height.
// All samples and outputs belong to a single control cycle; no servo PWM here.
namespace AP_CustomControl_MIMO {

enum class VerticalReference : uint8_t { HEIGHT, CLIMB_RATE };
enum class DisengageReason : uint8_t { SWITCH, INELIGIBLE, INVALID_INPUT, INVALID_OUTPUT };

struct Demand {
    float differential_brake; // positive requests right brake
    float common_brake;
    float throttle;

    bool finite() const
    {
        return isfinite(differential_brake) && isfinite(common_brake) && isfinite(throttle);
    }
};

struct Reference {
    uint32_t time_ms;
    bool valid;
    VerticalReference vertical;
    float height_home_m;
    float climb_rate_mps;
    float heading_rate_rads;
    float equivalent_airspeed_mps;
};

struct State {
    uint32_t time_ms;
    uint32_t reset_sequence;
    bool attitude_valid;
    bool navigation_valid;
    bool airspeed_valid;
    bool wind_valid;
    Quaternion attitude_body_to_ned;
    Vector3f body_rate_rads;
    Vector3f velocity_ned_mps;
    Vector3f wind_ned_mps;
    float height_home_m;
    float true_airspeed_mps;
    float eas_to_tas;
    float battery_voltage;
};

struct Authority {
    float common_brake_max;
    float differential_brake_max;
    float throttle_min;
    float throttle_max;
    float brake_slew_per_s;
    float throttle_slew_per_s;
};

enum Limit : uint16_t {
    BRAKE_TRAVEL = 1U << 0,
    BRAKE_SLEW = 1U << 1,
    THROTTLE_TRAVEL = 1U << 2,
    THROTTLE_SLEW = 1U << 3,
    VEHICLE_OVERRIDE = 1U << 4,
};

struct Allocation {
    uint32_t time_ms;
    Demand requested;
    Demand applied;
    uint16_t limits;
};

struct Input {
    uint32_t time_ms;
    float dt;
    Reference reference;
    State state;
    Authority authority;
    Allocation previous;
    uint8_t flight_mode;
    uint8_t flight_stage;
    bool eligible;
};

// Coordinate limits apply to actual left/right brakes, including shared travel.
// Return false rather than quietly accepting a partial/nonfinite command vector.
bool allocate(const Demand &request, const Demand &previous,
              const Authority &authority, float dt, Allocation &result);

} // namespace AP_CustomControl_MIMO

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#ifndef AP_PLANE_TRAJECTORY_ENABLED
#define AP_PLANE_TRAJECTORY_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_PLANE_TRAJECTORY_ENABLED
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

// Experimental, bounded-work wind-aware waypoint transition planner.
class AP_PlaneTrajectory {
public:
    AP_PlaneTrajectory();
    static const AP_Param::GroupInfo var_info[];
    bool enabled() const { return _enable.get() != 0; }
    bool configure_response(float period, float damping, float angle_p, float roll_time_constant,
                            float roll_rate, float roll_accel, float bank_limit);
    float transition_time() const { return _margin; }
    bool active() const { return _active && _tracking; }
    bool matches(uint16_t index, const Location &target) const;
    bool planned() const { return _active; }
    // The outgoing target is a boundary, never a rounded corner.
    bool exit_target(uint16_t index) const { return _active && index == _first_index + _corners; }
    void reset() { _active = false; _tracking = false; _last_index = 0; }
    // points: previous waypoint, up to three corners, subsequent waypoint.
    bool plan(const Location *points, uint8_t corners,
              uint16_t first_index, const Vector2f &wind, float airspeed, float bank_limit,
              const Location *position = nullptr, const Vector2f *velocity = nullptr, float bank_deg = 0,
              float out_extension = 0);
    bool entry_mismatch(const Location &position, const Vector2f &velocity, float bank_deg) const;
    bool turn_distances(const Location &previous, const Location &corner, const Location &next,
                        const Vector2f &wind, float airspeed, float bank_limit, float &entry, float &exit) const;
    bool update(const Location &position, const Vector2f &velocity, uint16_t index,
                float &bank_cd, bool &complete);
    bool attempted(uint16_t index) const { return _last_index == index; }
    float bank_cd() const { return _bank_cd; }
    float crosstrack_error_m() const { return _xtrack; }
    int32_t nav_bearing_cd() const { return _nav_bearing_cd; }
private:
    friend class PlaneTrajectoryTest;
    struct Path {
        Vector2f start;
        float heading;
        float time[4];
        float rate[4];
        float duration() const { return time[0] + time[1] + time[2] + time[3]; }
    };
    struct State {
        Vector2f position;
        Vector2f velocity;
        float heading;
        float rate;
    };
    State sample(const Path &path, float t) const;
    float path_cost(const Path &path) const;
    bool heading_for_course(const Vector2f &direction, float &heading) const;
    bool solve_line(const Vector2f &in_dir, float in_limit, const Vector2f &out_dir,
                    float out_limit, float start_heading, float end_heading,
                    const Vector2f *fixed_start);
    bool within_turn_space(const Path &path, const Vector2f &in_dir, float in_limit,
                           const Vector2f &out_dir, float out_limit) const;
    bool corner_times(const Path &path, const Vector2f *gates,
                     uint8_t corners, float *times) const;
    static float planning_bank(float bank_limit);
    static float planning_rate(float airspeed, float bank_limit);
    void capture_tolerances(float speed, float &cross, float &course) const;
    AP_Int8 _enable;
    AP_Float _extent;
    float _period = 8;
    float _damping = 0.9f;
    float _roll_tau = 0.5f;
    float _margin = 1.5f;
    float _bank_limit = 45;
    Location _origin;
    Location _targets[4]{};
    Vector2f _wind;
    float _airspeed = 0;
    float _omega = 0;
    Path _path{};
    float _corner_time[3]{};
    Vector2f _corner_position[3]{};
    float _tracking_limit = 100;
    uint16_t _first_index = 0;
    uint16_t _last_index = 0;
    uint8_t _corners = 0;
    float _progress = 0;
    float _bank_cd = 0;
    float _xtrack = 0;
    int32_t _nav_bearing_cd = 0;
    bool _active = false;
    bool _tracking = false;
    bool _state_entry = false;
    uint32_t _capture_start_ms = 0;
};
#endif // AP_PLANE_TRAJECTORY_ENABLED

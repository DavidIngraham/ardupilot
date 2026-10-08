#include "PlaneTrajectory.h"
#if AP_PLANE_TRAJECTORY_ENABLED
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_PlaneTrajectory::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Experimental waypoint trajectory guidance
    // @Description: Connect incoming and outgoing ground-track lines using wind-aware turns. Ordinary waypoint acceptance radii do not constrain planned turns or mission handover. Overlapping turns are grouped; explicit pass-by distances remain protected. Unsupported or infeasible transitions use normal L1 guidance. This prototype is disabled by default.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_PlaneTrajectory, _enable, 0, AP_PARAM_FLAG_ENABLE),
    // Indices 1-3 were RATE, PERIOD and MARGIN; do not reuse.
    // @Param: EXTENT
    // @DisplayName: Waypoint turn space
    // @Description: Maximum distance before the first corner and after the last corner that may be used by a planned turn. The turn also stays inside these along-track boundaries. Zero derives the space from turning capability and available leg length. Set to the turnaround extension when protecting survey legs. Independent of waypoint acceptance radius.
    // @Range: 0 1000
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("EXTENT", 4, AP_PlaneTrajectory, _extent, 0),
    AP_GROUPEND
};

AP_PlaneTrajectory::AP_PlaneTrajectory()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float AP_PlaneTrajectory::planning_bank(float bank_limit)
{
    // Leave bank authority for feedback around the nominal trajectory.
    return bank_limit * 0.7f;
}

float AP_PlaneTrajectory::planning_rate(float airspeed, float bank_limit)
{
    return GRAVITY_MSS * tanf(radians(planning_bank(bank_limit))) / airspeed;
}

bool AP_PlaneTrajectory::configure_response(float period, float damping, float angle_p, float roll_time_constant,
                                           float roll_rate, float roll_accel, float bank_limit)
{
    if (!isfinite(period) || !isfinite(damping) || !isfinite(angle_p) ||
        !isfinite(roll_time_constant) || !isfinite(roll_rate) || !isfinite(roll_accel) || !isfinite(bank_limit) ||
        period <= 0 || damping <= 0 || angle_p <= 0 || roll_time_constant <= 0 ||
        bank_limit <= 0 || bank_limit >= 85) {
        return false;
    }
    // Straight-line L1 linearisation has omega_n = 2*pi/period and the same
    // 2*zeta*omega_n velocity damping used by the trajectory error dynamics.
    _period = constrain_float(period, 1, 60);
    _damping = constrain_float(damping, 0.6f, 1);
    _roll_tau = MAX(0.05f, 1 / angle_p);
    const float reversal = 2 * planning_bank(bank_limit);
    const float residual = reversal * 0.05f;
    _margin = logf(20) * _roll_tau; // First-order response to 95% of a bank step.
    if (roll_rate > 0 && reversal > roll_rate * _roll_tau) {
        const float linear = (reversal - roll_rate * _roll_tau) / roll_rate;
        const float settling = _roll_tau * logf(MAX(1.0f, roll_rate * _roll_tau / residual));
        _margin = MAX(_margin, linear + settling);
    }
    if (roll_accel > 0) {
        // Bounds for rest-to-rest acceleration- and jerk-limited bank reversal.
        const float jerk = roll_accel / MAX(roll_time_constant, 0.1f);
        _margin = MAX(_margin, MAX(2 * sqrtf(reversal / roll_accel),
                                  4 * cbrtf(reversal / (2 * jerk))));
        if (roll_rate > 0 && reversal > sq(roll_rate) / roll_accel) {
            _margin = MAX(_margin, reversal / roll_rate + roll_rate / roll_accel);
        }
    }
    return true;
}

void AP_PlaneTrajectory::capture_tolerances(float speed, float &cross, float &course) const
{
    const float frequency = 2 * M_PI / _period;
    const float reserve = GRAVITY_MSS * (tanf(radians(_bank_limit)) -
                                        tanf(radians(planning_bank(_bank_limit))));
    // Require each residual error to be recoverable with the bank authority
    // reserved for feedback. Neither tolerance depends on waypoint radius.
    cross = reserve / sq(frequency);
    course = asinf(constrain_float(reserve / (2 * _damping * frequency * MAX(speed, 3.0f)), 0, 1));
}

AP_PlaneTrajectory::State AP_PlaneTrajectory::sample(const Path &path, float t) const
{
    State s{path.start, {}, path.heading, 0};
    for (uint8_t i = 0; i < 4; i++) {
        const float dt = constrain_float(t, 0, path.time[i]);
        const float r = path.rate[i];
        const float end_heading = s.heading + r * dt;
        if (fabsf(r) > 0.001f) {
            s.position += Vector2f(sinf(end_heading) - sinf(s.heading),
                                   cosf(s.heading) - cosf(end_heading)) * (_airspeed / r);
        } else {
            s.position += Vector2f(cosf(s.heading), sinf(s.heading)) * (_airspeed * dt);
        }
        s.position += _wind * dt;
        s.heading = end_heading;
        s.rate = r;
        t -= dt;
        if (t <= 0) {
            break;
        }
    }
    s.velocity = Vector2f(cosf(s.heading), sinf(s.heading)) * _airspeed + _wind;
    return s;
}

float AP_PlaneTrajectory::path_cost(const Path &path) const
{
    float duration = 0;
    float turn = 0;
    float heading_change = 0;
    for (uint8_t i = 0; i < 4; i++) {
        duration += path.time[i];
        const float angle = path.rate[i] * path.time[i];
        turn += fabsf(angle);
        heading_change += angle;
    }
    // Charge excess turning its equivalent time at the planning rate. This
    // discourages loops without excluding a feasible turn by its angle.
    const float extra_turn = MAX(0.0f, turn - fabsf(wrap_PI(heading_change)));
    return duration + extra_turn / _omega;
}

bool AP_PlaneTrajectory::matches(uint16_t index, const Location &target) const
{
    if (!_active || index < _first_index || index > _first_index + _corners) {
        return false;
    }
    const Location &saved = _targets[index - _first_index];
    return saved.same_latlon_as(target) && saved.alt == target.alt &&
           saved.relative_alt == target.relative_alt && saved.terrain_alt == target.terrain_alt;
}

bool AP_PlaneTrajectory::heading_for_course(const Vector2f &direction, float &heading) const
{
    const float crosswind = direction.x * _wind.y - direction.y * _wind.x;
    if (fabsf(crosswind) >= _airspeed * 0.95f) {
        return false;
    }
    heading = atan2f(direction.y, direction.x) - asinf(crosswind / _airspeed);
    const Vector2f velocity = Vector2f(cosf(heading), sinf(heading)) * _airspeed + _wind;
    return velocity * direction > 2;
}

// Solve d_in*u_in + d_out*u_out = integral(V_air + wind) dt.
// These tangent distances depend on aircraft capability and wind, not waypoint radius.
bool AP_PlaneTrajectory::turn_distances(const Location &previous, const Location &corner,
                                        const Location &next, const Vector2f &wind,
                                        float airspeed, float bank_limit, float &entry, float &exit) const
{
    const Vector2f incoming = previous.get_distance_NE(corner);
    const Vector2f outgoing = corner.get_distance_NE(next);
    if (incoming.length() < 1 || outgoing.length() < 1 || airspeed < 5 ||
        !isfinite(airspeed) || !isfinite(bank_limit) || bank_limit <= 0 || bank_limit >= 85 || !isfinite(wind.x) || !isfinite(wind.y)) {
        return false;
    }
    const Vector2f a = incoming.normalized();
    const Vector2f b = outgoing.normalized();
    const float determinant = a.x * b.y - a.y * b.x;
    if (fabsf(determinant) < 0.01f) {
        return false;
    }
    const float cross1 = a.x * wind.y - a.y * wind.x;
    const float cross2 = b.x * wind.y - b.y * wind.x;
    if (fabsf(cross1) >= airspeed * 0.95f || fabsf(cross2) >= airspeed * 0.95f) {
        return false;
    }
    const float h1 = atan2f(a.y, a.x) - asinf(cross1 / airspeed);
    const float h2 = atan2f(b.y, b.x) - asinf(cross2 / airspeed);
    const float angle = wrap_PI(h2 - h1);
    const float omega = planning_rate(airspeed, bank_limit);
    const float rate = angle > 0 ? omega : -omega;
    const Vector2f displacement = Vector2f(sinf(h2) - sinf(h1), cosf(h1) - cosf(h2)) *
                                  (airspeed / rate) + wind * (fabsf(angle) / omega);
    entry = (displacement.x * b.y - displacement.y * b.x) / determinant;
    exit = (a.x * displacement.y - a.y * displacement.x) / determinant;
    return isfinite(entry) && isfinite(exit) && entry >= 0 && exit >= 0;
}

bool AP_PlaneTrajectory::corner_times(const Path &path, const Vector2f *gates,
                                    uint8_t corners, float *times) const
{
    const float duration = path.duration();
    float previous = 0;
    for (uint8_t g = 0; g < corners; g++) {
        float distance = FLT_MAX;
        float best = previous;
        // Ordered reference-corner progress determines mission handover; no radius gate.
        const float step = MAX(0.1f, duration / 400);
        for (float t = previous; t <= duration; t += step) {
            const float d = (sample(path, t).position - gates[g]).length();
            if (d < distance) {
                distance = d;
                best = t;
            }
        }
        times[g] = MAX(MIN(best, duration - 0.2f), previous + 0.2f);
        if (times[g] >= duration - 0.1f) {
            return false;
        }
        previous = times[g];
    }
    // Keep the last corner active until the complete join has been traversed.
    times[corners - 1] = MAX(times[corners - 1], duration - MIN(0.5f, duration * 0.1f));
    return true;
}

bool AP_PlaneTrajectory::within_turn_space(const Path &path, const Vector2f &in_dir,
                                          float in_limit, const Vector2f &out_dir,
                                          float out_limit) const
{
    const float out_bound = _corner_position[_corners - 1] * out_dir + out_limit;
    auto inside = [&](float t) {
        const Vector2f p = sample(path, t).position;
        return p * in_dir >= -in_limit - 0.1f && p * out_dir <= out_bound + 0.1f;
    };
    float elapsed = 0;
    float heading = path.heading;
    if (!inside(0)) {
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) {
        const float rate = path.rate[i];
        const float next_heading = heading + rate * path.time[i];
        if (fabsf(rate) > 0.001f) {
            // Projection extrema occur where ground velocity is perpendicular
            // to a boundary normal. Check them exactly, not at sampled positions.
            for (const Vector2f normal : {in_dir, out_dir}) {
                const float q = -(normal * _wind) / _airspeed;
                if (fabsf(q) <= 1) {
                    const float phi = atan2f(normal.y, normal.x);
                    const float theta = acosf(q);
                    for (const float sign : {-1.0f, 1.0f}) {
                        for (int8_t winding = -3; winding <= 3; winding++) {
                            const float h = phi + sign * theta + winding * 2 * M_PI;
                            const float dt = (h - heading) / rate;
                            if (dt > 0 && dt < path.time[i] && !inside(elapsed + dt)) {
                                return false;
                            }
                        }
                    }
                }
            }
        }
        elapsed += path.time[i];
        heading = next_heading;
        if (!inside(elapsed)) {
            return false;
        }
    }
    return true;
}

bool AP_PlaneTrajectory::solve_line(const Vector2f &in_dir, float in_limit,
                                   const Vector2f &out_dir, float out_limit,
                                   float start_heading, float end_heading,
                                   const Vector2f *fixed_start)
{
    const float revolution = 2 * M_PI / _omega;
    const float difference = wrap_PI(end_heading - start_heading);
    const float margin = _airspeed * _margin;
    const float in_speed = (Vector2f(cosf(start_heading), sinf(start_heading)) * _airspeed + _wind) * in_dir;
    const float out_speed = (Vector2f(cosf(end_heading), sinf(end_heading)) * _airspeed + _wind) * out_dir;
    const float entry_min = -in_limit;
    const float entry_max = in_limit - margin;
    const float exit_min = -out_limit;
    const float exit_max = out_limit - margin;
    if (entry_max < entry_min || exit_max < exit_min || in_speed <= 2 || out_speed <= 2) {
        return false;
    }
    float best_cost = FLT_MAX;
    Path best_path{};
    float best_times[3]{};
    for (int8_t first = -1; first <= 1; first += 2) {
        for (int8_t last = -1; last <= 1; last += 2) {
            for (int8_t winding = -2; winding <= 2; winding++) {
                const float angle = difference + winding * 2 * M_PI;
                float family_cost = FLT_MAX;
                float best_entry = 0;
                float best_t1 = 0;
                auto evaluate = [&](float entry, float t1) {
                    if (entry < entry_min || entry > entry_max || t1 < 0 || t1 > revolution) {
                        return;
                    }
                    const float t3 = (angle - first * _omega * t1) / (last * _omega);
                    if (t3 < 0 || t3 > revolution) {
                        return;
                    }
                    Path p{fixed_start == nullptr ? -in_dir * entry : *fixed_start,
                           start_heading, {t1, 0, t3, margin / out_speed}, {first * _omega, 0, last * _omega, 0}};
                    const Vector2f base = sample(p, t1 + t3).position;
                    const float h = start_heading + first * _omega * t1;
                    const Vector2f v = Vector2f(cosf(h), sinf(h)) * _airspeed + _wind;
                    const Vector2f delta = _corner_position[_corners - 1] - base;
                    const float denominator = out_dir.x * v.y - out_dir.y * v.x;
                    if (fabsf(denominator) < 0.01f) {
                        return;
                    }
                    // Terminal position is free along the outgoing line. The
                    // transverse equation alone determines the straight time.
                    p.time[1] = (out_dir.x * delta.y - out_dir.y * delta.x) / denominator;
                    const float duration = p.duration();
                    if (p.time[1] < 0 || duration >= 120 || !isfinite(duration)) {
                        return;
                    }
                    const Vector2f end = sample(p, duration).position;
                    const float exit = (end - _corner_position[_corners - 1]) * out_dir;
                    if (exit < exit_min || exit > exit_max) {
                        return;
                    }
                    const float actual_entry = -(p.start * in_dir);
                    const float cost = path_cost(p) - actual_entry / in_speed - exit / out_speed;
                    if (cost >= family_cost ||
                        !within_turn_space(p, in_dir, in_limit - margin, out_dir, out_limit - margin)) {
                        return;
                    }
                    float times[3]{};
                    if (!corner_times(p, _corner_position, _corners, times)) {
                        return;
                    }
                    family_cost = cost;
                    best_entry = entry;
                    best_t1 = t1;
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_path = p;
                        memcpy(best_times, times, sizeof(times));
                    }
                };
                // Bounded global seeds, then continuous two-dimensional refinement.
                for (uint8_t entry = 0; entry < (fixed_start == nullptr ? 5 : 1); entry++) {
                    const float offset = entry_min + (entry_max - entry_min) * entry / 4;
                    for (uint8_t j = 0; j <= 64; j++) {
                        evaluate(offset, revolution * j / 64);
                    }
                }
                if (family_cost >= FLT_MAX) {
                    continue;
                }
                float entry_step = (entry_max - entry_min) / 4;
                float time_step = revolution / 64;
                for (uint8_t iteration = 0; iteration < 16; iteration++) {
                    const float old_cost = family_cost;
                    const float entry = best_entry;
                    const float t1 = best_t1;
                    for (int8_t a = -1; a <= 1; a++) {
                        for (int8_t b = -1; b <= 1; b++) {
                            evaluate(fixed_start == nullptr ? entry + a * entry_step : entry,
                                     t1 + b * time_step);
                        }
                    }
                    if (family_cost >= old_cost) {
                        entry_step *= 0.5f;
                        time_step *= 0.5f;
                    }
                }
            }
        }
    }
    if (best_cost >= FLT_MAX) {
        return false;
    }
    _path = best_path;
    memcpy(_corner_time, best_times, sizeof(best_times));
    return true;
}

bool AP_PlaneTrajectory::entry_mismatch(const Location &position, const Vector2f &velocity,
                                       float bank_deg) const
{
    if (!_active || _tracking || _state_entry || velocity.length() < 3) {
        return false;
    }
    const State start = sample(_path, 0);
    const Vector2f pos = _origin.get_distance_NE(position);
    const Vector2f direction = start.velocity.normalized();
    const float margin = _airspeed * _margin;
    const float along = (pos - start.position) * direction;
    // Once past entry, retain the maneuver and recover with feedback. Starting
    // a new tangent solution there can switch topology for a small state error.
    if (along < -margin || along > 0) {
        return false;
    }
    const float cross = direction.x * (pos.y - start.position.y) - direction.y * (pos.x - start.position.x);
    const float course = wrap_PI(atan2f(velocity.y, velocity.x) - atan2f(start.velocity.y, start.velocity.x));
    float cross_tolerance = 0;
    float course_tolerance = 0;
    capture_tolerances(velocity.length(), cross_tolerance, course_tolerance);
    return fabsf(cross) > cross_tolerance || fabsf(course) > course_tolerance ||
           fabsf(bank_deg) > _bank_limit - planning_bank(_bank_limit);
}

bool AP_PlaneTrajectory::plan(const Location *points, uint8_t corners,
                              uint16_t first_index, const Vector2f &wind, float airspeed, float bank_limit,
                              const Location *position, const Vector2f *velocity, float bank_deg,
                              float out_extension)
{
    reset();
    _last_index = first_index;
    if (corners < 1 || corners > 3 || !isfinite(airspeed) || airspeed < 5 ||
        !isfinite(bank_limit) || bank_limit <= 0 || bank_limit >= 85 || !isfinite(wind.x) || !isfinite(wind.y) ||
        !isfinite(out_extension) || out_extension < 0 || wind.length() > airspeed * 0.85f) {
        return false;
    }
    _corners = corners;
    _state_entry = position != nullptr && velocity != nullptr;
    _origin = points[1];
    _wind = wind;
    _airspeed = airspeed;
    _omega = planning_rate(airspeed, bank_limit);
    _bank_limit = bank_limit;
    const float radius = airspeed / _omega;
    const Vector2f incoming = points[0].get_distance_NE(points[1]);
    const Vector2f outgoing = points[corners].get_distance_NE(points[corners + 1]);
    if (incoming.length() < 20 || outgoing.length() < 20) {
        return false;
    }
    const Vector2f in_dir = incoming.normalized();
    const Vector2f out_dir = outgoing.normalized();
    float h1 = 0;
    float h2 = 0;
    if (!heading_for_course(in_dir, h1) || !heading_for_course(out_dir, h2)) {
        return false;
    }
    for (uint8_t g = 0; g < corners; g++) {
        _corner_position[g] = _origin.get_distance_NE(points[g + 1]);
    }
    const float margin = airspeed * _margin;
    const float in_limit = MIN(incoming.length() - 10,
                               _extent.get() > 0 ? _extent.get() : 4 * radius + margin);
    const float out_limit = MIN(outgoing.length() + out_extension - 10,
                                _extent.get() > 0 ? _extent.get() : 4 * radius + margin);
    Vector2f predicted_start;
    const Vector2f *fixed_start = nullptr;
    if (position != nullptr && velocity != nullptr) {
        if (!isfinite(bank_deg) || !isfinite(velocity->x) || !isfinite(velocity->y)) {
            return false;
        }
        const Vector2f air_velocity = *velocity - wind;
        if (air_velocity.length() < 5) {
            return false;
        }
        h1 = atan2f(air_velocity.y, air_velocity.x);
        // Predict arrival after unwinding the measured bank over the roll
        // allowance, using its mean heading rate rather than assuming level flight.
        const float dt = _margin;
        const float rate = GRAVITY_MSS * tanf(radians(constrain_float(bank_deg, -bank_limit, bank_limit))) /
                           (2 * airspeed);
        Path coast{_origin.get_distance_NE(*position), h1, {dt, 0, 0}, {rate, 0, 0}};
        const State entry = sample(coast, dt);
        predicted_start = entry.position;
        h1 = entry.heading;
        fixed_start = &predicted_start;
    }
    float first_entry = 0;
    float first_exit = 0;
    const bool single_tangent = turn_distances(points[0], points[1], points[2], wind, airspeed,
                                              bank_limit, first_entry, first_exit);
    if (!_state_entry && corners == 1 && single_tangent &&
        first_entry + margin < in_limit && first_exit < out_limit) {
        const float in_speed = (Vector2f(cosf(h1), sinf(h1)) * airspeed + wind) * in_dir;
        const float out_speed = (Vector2f(cosf(h2), sinf(h2)) * airspeed + wind) * out_dir;
        const float angle = wrap_PI(h2 - h1);
        const float exit_margin = MIN(margin, out_limit - first_exit);
        _path = {-in_dir * (first_entry + margin), h1,
                 {margin / in_speed, fabsf(angle) / _omega, exit_margin / out_speed},
                 {0, angle > 0 ? _omega : -_omega, 0}};
        if (!corner_times(_path, _corner_position, corners, _corner_time)) {
            return false;
        }
    } else if (!solve_line(in_dir, in_limit, out_dir, out_limit, h1, h2, fixed_start)) {
        return false;
    }
    _tracking_limit = MAX(100.0f, radius * 2);
    for (uint8_t i = 0; i <= corners; i++) {
        _targets[i] = points[i + 1];
    }
    _first_index = first_index;
    _corners = corners;
    _progress = 0;
    _capture_start_ms = 0;
    _active = true;
    return true;
}

bool AP_PlaneTrajectory::update(const Location &position, const Vector2f &velocity,
                                uint16_t index, float &bank_cd, bool &complete)
{
    complete = false;
    if (!_active || index < _first_index || index > _first_index + _corners || !isfinite(velocity.x) ||
        !isfinite(velocity.y) || velocity.length() < 3) {
        _active = false;
        return false;
    }
    const Vector2f pos = _origin.get_distance_NE(position);
    const Vector2f entry_direction = sample(_path, 0).velocity.normalized();
    const float entry_along = (pos - _path.start) * entry_direction;
    const float entry_lead = _airspeed * _margin;
    if (is_zero(_progress) && entry_along < -entry_lead) {
        _tracking = false;
        return false;
    }
    _tracking = true;
    const float duration = _path.duration();
    float best_distance = FLT_MAX;
    float nearest = _progress;
    // Monotone bounded projection prevents jumping across crossing paths.
    for (float t = MAX(0, _progress - 0.3f); t <= MIN(duration, _progress + 5); t += 0.05f) {
        const float d = (sample(_path, t).position - pos).length_squared();
        if (d < best_distance) {
            best_distance = d;
            nearest = t;
        }
    }
    _progress = MAX(_progress, nearest);
    const State s = sample(_path, _progress);
    const Vector2f direction = s.velocity.normalized();
    const Vector2f error = pos - s.position;
    const float cross = direction.x * error.y - direction.y * error.x;
    const float course_error = wrap_PI(atan2f(s.velocity.y, s.velocity.x) - atan2f(velocity.y, velocity.x));
    _xtrack = cross;
    _nav_bearing_cd = wrap_180_cd(degrees(atan2f(s.velocity.y, s.velocity.x)) * 100);
    const float frequency = 2 * M_PI / _period;
    const float correction = -sq(frequency) * cross + 2 * _damping * frequency * velocity.length() *
                             sinf(constrain_float(course_error, -M_PI_2, M_PI_2));
    const Vector2f air_dir(cosf(s.heading), sinf(s.heading));
    const float projection = MAX(0.3f, air_dir * direction);
    // Preview the rate transition to compensate normal Plane bank-response lag.
    const float preview_rate = ((_progress + 0.5f * _roll_tau >= duration ? 0 : sample(_path, _progress + 0.5f * _roll_tau).rate) +
                                (_progress + 1.5f * _roll_tau >= duration ? 0 : sample(_path, _progress + 1.5f * _roll_tau).rate)) * 0.5f;
    const float entry_scale = entry_lead > 0 && is_zero(_progress) ?
        constrain_float((entry_along + entry_lead) / entry_lead, 0, 1) : 1;
    const float heading_rate = preview_rate * entry_scale + correction / (_airspeed * projection);
    _bank_cd = degrees(atanf(_airspeed * heading_rate / GRAVITY_MSS)) * 100;
    bank_cd = _bank_cd;
    const uint8_t gate = index - _first_index;
    float cross_tolerance = 0;
    float course_tolerance = 0;
    capture_tolerances(velocity.length(), cross_tolerance, course_tolerance);
    complete = gate < _corners && _progress >= _corner_time[gate] &&
               (gate + 1 < _corners ||
                (fabsf(cross) < cross_tolerance && velocity * direction > velocity.length() * cosf(course_tolerance)));
    // An untracked path must not trap the mission indefinitely; reacquire using L1.
    const bool at_end = _progress >= duration - 0.1f;
    if (at_end && gate < _corners && !complete) {
        if (_capture_start_ms == 0) {
            _capture_start_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - _capture_start_ms > uint32_t(MAX(10.0f, 4 * _period) * 1000)) {
            _active = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Trajectory capture fallback WP %u", index);
            return false;
        }
    }
    if (!at_end && best_distance > sq(_tracking_limit)) {
        _active = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Trajectory tracking fallback WP %u", index);
        return false;
    }
    if (at_end && gate >= _corners) {
        _active = false;
        return false;
    }
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: NTP
    // @Description: Experimental waypoint trajectory tracking
    // @Field: TimeUS: Time since system startup
    // @Field: Index: Active mission waypoint index
    // @Field: Count: Number of corners in this transition
    // @Field: Prog: Projected trajectory time in seconds
    // @Field: End: Total trajectory time in seconds
    // @Field: XTrack: Signed cross-track error in metres
    // @Field: Rate: Planned heading rate in degrees per second
    // @Field: Roll: Requested bank in degrees
    AP::logger().WriteStreaming("NTP", "TimeUS,Index,Count,Prog,End,XTrack,Rate,Roll", "QHBfffff",
                               AP_HAL::micros64(), index, _corners, _progress, duration, cross, degrees(preview_rate), _bank_cd * 0.01f);
#endif
    return true;
}
#endif // AP_PLANE_TRAJECTORY_ENABLED

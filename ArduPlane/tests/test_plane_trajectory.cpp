#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_Dummy.h>
// Planner tests do not run the vehicle logger.
#undef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#include "../PlaneTrajectory.cpp"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
GCS_Dummy dummy_gcs;

#if AP_PLANE_TRAJECTORY_ENABLED
class PlaneTrajectoryTest : public ::testing::Test {
protected:
    static Location location(float north, float east)
    {
        Location loc{-353629380, 1491650850, 10000, Location::AltFrame::ABOVE_HOME};
        loc.offset(north, east);
        return loc;
    }
    static void verify_endpoint(AP_PlaneTrajectory &planner, const Location *points, uint8_t corners)
    {
        const auto &p = planner._path;
        const float t = p.duration();
        const auto end = planner.sample(p, t);
        if (corners == 1) {
            EXPECT_LT(t, 15);
        }
        float heading = 0;
        const Vector2f outgoing = points[corners].get_distance_NE(points[corners + 1]).normalized();
        ASSERT_TRUE(planner.heading_for_course(outgoing, heading));
        EXPECT_NEAR(wrap_PI(end.heading - heading), 0, 0.001f);
        const Vector2f delta = end.position - planner._corner_position[corners - 1];
        EXPECT_NEAR(delta.x * outgoing.y - delta.y * outgoing.x, 0, 0.5f);
        EXPECT_GT(delta * outgoing, 0);
        for (uint8_t i = 0; i < corners; i++) {
            if (i > 0) {
                EXPECT_GT(planner._corner_time[i], planner._corner_time[i - 1]);
            }
        }
    }
    static void verify_exit_boundary()
    {
        const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
            AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
        ASSERT_TRUE(planner.exit_target(3));
        EXPECT_FALSE(planner.exit_target(2));
        const float end = planner._path.duration();
        bool handled = false;
        for (float t = 0; t < end - 0.2f; t += 0.1f) {
            const auto state = planner.sample(planner._path, t);
            Location position = planner._origin;
            position.offset(state.position.x, state.position.y);
            bool complete = true;
            float bank = 0;
            handled |= planner.update(position, state.velocity, 3, bank, complete);
            EXPECT_FALSE(complete); // Incoming turn must not accept the protected outgoing waypoint.
        }
        EXPECT_TRUE(handled);
    }
    static void verify_acute_line_transition()
    {
        const Location points[]{location(-600, 0), location(0, 0), location(-424, 424)};
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
        verify_endpoint(planner, points, 1);
        const float closest = (planner.sample(planner._path, planner._corner_time[0]).position -
                               planner._corner_position[0]).length();
        EXPECT_GT(closest, 50); // A valid tangent turn must not be rejected by the old radius gate.
        float entry = 0;
        float exit = 0;
        ASSERT_TRUE(planner.turn_distances(points[0], points[1], points[2], Vector2f(0, 0), 20, 45, entry, exit));
        EXPECT_NEAR(entry, exit, 0.1f);
        EXPECT_GT(entry, 100);
    }

    static void verify_dense_upwind_reversal()
    {
        const Location points[]{location(-1000, 0), location(0, 0),
                                location(0, 40), location(-1000, 40)};
        for (const float airspeed : {21.0f, 22.0f, 23.0f}) {
            AP_PlaneTrajectory planner;
            planner._extent.set(200);
            ASSERT_TRUE(planner.plan(points, 2, 3, Vector2f(0, -10), airspeed, 45));
            verify_endpoint(planner, points, 2);
            float turn = 0;
            for (uint8_t i = 0; i < 4; i++) {
                EXPECT_GE(planner._path.time[i], 0);
                turn += fabsf(planner._path.rate[i]) * planner._path.time[i];
                EXPECT_LE(fabsf(planner._path.rate[i]), planner._omega + 0.001f);
            }
            const float duration = planner._path.duration();
            EXPECT_LT(duration, 120);
            EXPECT_GE(planner.path_cost(planner._path), duration);
            EXPECT_TRUE(planner.within_turn_space(planner._path, Vector2f(1, 0), 200,
                                                  Vector2f(-1, 0), 200));
            for (float t = 0; t < duration; t += 0.05f) {
                EXPECT_GE(planner.sample(planner._path, t).position.x, -200.1f);
            }
        }
    }

    static void verify_excess_turn_cost()
    {
        AP_PlaneTrajectory planner;
        planner._omega = radians(20);
        const AP_PlaneTrajectory::Path direct{{}, 0, {2, 0, 2},
                                              {planner._omega, 0, planner._omega}};
        const AP_PlaneTrajectory::Path reversing{{}, 0, {2, 0, 2},
                                                 {planner._omega, 0, -planner._omega}};
        EXPECT_NEAR(planner.path_cost(direct), 4, 0.001f);
        EXPECT_NEAR(planner.path_cost(reversing), 8, 0.001f);
    }

    static void verify_measured_entry()
    {
        const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
        const Location position = location(-100, 12);
        const Vector2f velocity(20 * cosf(radians(15)), 20 * sinf(radians(15)));
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45, &position, &velocity, 10));
        EXPECT_GT(degrees(planner._path.heading), 15);
        EXPECT_GT(planner._path.start.y, 12);
        EXPECT_FALSE(planner.entry_mismatch(position, velocity, 10));
        const float duration = planner._path.duration();
        const auto end = planner.sample(planner._path, duration);
        EXPECT_NEAR(end.position.x, 0, 0.1f);
        EXPECT_NEAR(wrap_PI(end.heading - M_PI_2), 0, 0.001f);
    }

    static void verify_curve_boundary_extremum()
    {
        AP_PlaneTrajectory planner;
        planner._airspeed = 20;
        planner._omega = 0.3f;
        planner._corners = 1;
        // Endpoints fit, but a complete circle crosses the entry boundary.
        const AP_PlaneTrajectory::Path loop{{-100, 0}, M_PI_2,
                                            {2 * M_PI / planner._omega, 0, 0},
                                            {planner._omega, 0, 0}};
        EXPECT_FALSE(planner.within_turn_space(loop, Vector2f(1, 0), 150,
                                               Vector2f(0, 1), 200));
        EXPECT_TRUE(planner.within_turn_space(loop, Vector2f(1, 0), 250,
                                              Vector2f(0, 1), 200));
    }

    static void verify_free_line_exit()
    {
        const Location points[]{location(-1000, 0), location(0, 0),
                                location(0, 40), location(-1000, 40)};
        float largest_departure = 0;
        for (const float airspeed : {21.0f, 22.0f, 23.0f}) {
            AP_PlaneTrajectory planner;
            planner._extent.set(200);
            ASSERT_TRUE(planner.plan(points, 2, 3, Vector2f(0, -10), airspeed, 45));
            const auto end = planner.sample(planner._path, planner._path.duration());
            const Vector2f outgoing = points[2].get_distance_NE(points[3]).normalized();
            const float exit = (end.position - planner._corner_position[1]) * outgoing;
            float entry = 0;
            float tangent_exit = 0;
            ASSERT_TRUE(planner.turn_distances(points[1], points[2], points[3], Vector2f(0, -10),
                                               airspeed, 45, entry, tangent_exit));
            float closest_old_exit = FLT_MAX;
            for (uint8_t i = 0; i < 4; i++) {
                const float old_exit = tangent_exit * (0.75f + i * 0.5f) + airspeed * 1.5f;
                closest_old_exit = MIN(closest_old_exit, fabsf(exit - old_exit));
            }
            largest_departure = MAX(largest_departure, closest_old_exit);
            EXPECT_LE(exit, 200);
        }
        EXPECT_GT(largest_departure, 0.5f);
    }

    static void verify_terminal_course_required()
    {
        const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
        float bank = 0;
        bool complete = false;
        for (float t = 0; t < planner._corner_time[0]; t += 0.05f) {
            const auto state = planner.sample(planner._path, t);
            Location position = planner._origin;
            position.offset(state.position.x, state.position.y);
            planner.update(position, state.velocity, 2, bank, complete);
        }
        const auto end = planner.sample(planner._path, planner._corner_time[0]);
        Location position = planner._origin;
        position.offset(end.position.x, end.position.y);
        ASSERT_TRUE(planner.update(position, -end.velocity, 2, bank, complete));
        EXPECT_FALSE(complete);
        ASSERT_TRUE(planner.update(position, end.velocity, 2, bank, complete));
        EXPECT_TRUE(complete);
    }

    static void verify_entry_replan_window()
    {
        const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
        const auto start = planner.sample(planner._path, 0);
        const Vector2f direction = start.velocity.normalized();
        Location before = planner._origin;
        const Vector2f p = start.position - direction * 10;
        before.offset(p.x, p.y);
        EXPECT_TRUE(planner.entry_mismatch(before, Vector2f(0, 20), 15));
        Location after = planner._origin;
        const Vector2f q = start.position + direction * 10;
        after.offset(q.x, q.y);
        EXPECT_FALSE(planner.entry_mismatch(after, Vector2f(0, 20), 15));
    }

    static void verify_terminal_line_capture()
    {
        const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
        float bank = 0;
        bool complete = false;
        for (float t = 0; t < planner._path.duration(); t += 0.05f) {
            const auto state = planner.sample(planner._path, t);
            Location position = planner._origin;
            position.offset(state.position.x, state.position.y);
            planner.update(position, state.velocity, 2, bank, complete);
        }
        const auto end = planner.sample(planner._path, planner._path.duration());
        Location position = planner._origin;
        const Vector2f beyond = end.position + end.velocity.normalized() * 40;
        position.offset(beyond.x, beyond.y);
        ASSERT_TRUE(planner.update(position, -end.velocity, 2, bank, complete));
        EXPECT_FALSE(complete);
        EXPECT_TRUE(planner.planned());
        EXPECT_GT(fabsf(bank), 100); // Recovery must also steer for a reversed course.
        ASSERT_TRUE(planner.update(position, end.velocity, 2, bank, complete));
        EXPECT_TRUE(complete); // The terminal condition is a line, not a fixed point.
    }

    static void verify_protected_finish_line_space()
    {
        const Location points[]{location(0, -410), location(0, 0), location(-90, 0)};
        for (const float airspeed : {20.0f, 21.0f, 22.0f, 23.0f}) {
            AP_PlaneTrajectory planner;
            ASSERT_TRUE(planner.plan(points, 1, 5, Vector2f(-3.5355f, -3.5355f), airspeed, 45,
                                     nullptr, nullptr, 0, 35));
            float turn = 0;
            for (uint8_t i = 0; i < 4; i++) {
                turn += fabsf(planner._path.rate[i]) * planner._path.time[i];
            }
            EXPECT_LT(turn, M_PI); // Small speed changes must not force a looping turn.
            const auto end = planner.sample(planner._path, planner._path.duration());
            EXPECT_NEAR(end.position.y, 0, 0.1f);
            EXPECT_GE(end.position.x, -125);
            EXPECT_LT(planner._path.duration(), 15);
        }
    }

    static void verify_derived_capability()
    {
        const Location points[]{location(-5000, 0), location(0, 0), location(0, 5000)};
        EXPECT_LT(AP_PlaneTrajectory::planning_bank(3), 3);
        for (const float bank : {10.0f, 30.0f, 60.0f}) {
            for (const float speed : {15.0f, 35.0f}) {
                AP_PlaneTrajectory planner;
                ASSERT_TRUE(planner.plan(points, 1, 2, {}, speed, bank));
                EXPECT_NEAR(planner._omega, GRAVITY_MSS * tanf(radians(bank * 0.7f)) / speed, 0.00001f);
                EXPECT_LT(degrees(atanf(planner._omega * speed / GRAVITY_MSS)), bank);
            }
        }
    }

    static void verify_derived_response()
    {
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.configure_response(12, 0.75f, 2, 0.5f, 0, 500, 45));
        EXPECT_FLOAT_EQ(planner._period, 12);
        EXPECT_FLOAT_EQ(planner._damping, 0.75f);
        EXPECT_NEAR(planner.transition_time(), logf(20) * 0.5f, 0.0001f);
        const float fast = planner.transition_time();
        ASSERT_TRUE(planner.configure_response(12, 0.75f, 2, 0.5f, 10, 500, 45));
        EXPECT_GT(planner.transition_time(), 2 * 45 * 0.7f / 10);
        ASSERT_TRUE(planner.configure_response(12, 0.75f, 1, 1, 0, 500, 45));
        EXPECT_NEAR(planner.transition_time(), 2 * fast, 0.0001f);
        ASSERT_TRUE(planner.configure_response(12, 0.75f, 2, 0.5f, 0, 10, 45));
        EXPECT_GE(planner.transition_time(), 2 * sqrtf(63.0f / 10));
        EXPECT_FALSE(planner.configure_response(NAN, 0.75f, 2, 0.5f, 0, 500, 45));
        EXPECT_FALSE(planner.configure_response(12, 0.75f, 0, 0.5f, 0, 500, 45));
        EXPECT_FALSE(planner.configure_response(12, 0.75f, 2, 0.5f, 0, 500, 0));
    }

    static void verify_capture_authority()
    {
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.configure_response(8, 0.9f, 2, 0.5f, 0, 500, 45));
        float cross = 0;
        float course = 0;
        planner.capture_tolerances(20, cross, course);
        const float reserve = GRAVITY_MSS * (tanf(radians(45)) - tanf(radians(31.5f)));
        const float omega = 2 * M_PI / 8;
        EXPECT_NEAR(sq(omega) * cross, reserve, 0.0001f);
        EXPECT_NEAR(2 * 0.9f * omega * 20 * sinf(course), reserve, 0.0001f);
        const float slow_course = course;
        planner.capture_tolerances(30, cross, course);
        EXPECT_LT(course, slow_course);
    }

};

TEST_F(PlaneTrajectoryTest, RightCornerCalmAndWind)
{
    const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
    for (const Vector2f wind : {Vector2f(0, 0), Vector2f(3, -4), Vector2f(-4, 3)}) {
        AP_PlaneTrajectory planner;
        ASSERT_TRUE(planner.plan(points, 1, 2, wind, 20, 45));
        verify_endpoint(planner, points, 1);
    }
}

TEST_F(PlaneTrajectoryTest, CapabilityUsesAirspeedAndBankWithoutIndependentRateCap)
{
    verify_derived_capability();
}

TEST_F(PlaneTrajectoryTest, ResponseUsesSharedNavigationAndRollSettings)
{
    verify_derived_response();
}

TEST_F(PlaneTrajectoryTest, CaptureErrorsFitReservedBankAuthority)
{
    verify_capture_authority();
}

TEST_F(PlaneTrajectoryTest, GroupedOppositeCorners)
{
    const Location points[]{location(-400, 0), location(0, 0), location(0, 90), location(400, 90)};
    AP_PlaneTrajectory planner;
    ASSERT_TRUE(planner.plan(points, 2, 2, Vector2f(0, 0), 20, 45));
    verify_endpoint(planner, points, 2);
}

TEST_F(PlaneTrajectoryTest, DenseUpwindReversalPreservesTurnSpace)
{
    verify_dense_upwind_reversal();
}

TEST_F(PlaneTrajectoryTest, MeasuredEntryPredictsCourseAndBank)
{
    verify_measured_entry();
}

TEST_F(PlaneTrajectoryTest, TurnSpaceChecksCurveExtrema)
{
    verify_curve_boundary_extremum();
}

TEST_F(PlaneTrajectoryTest, ExitIsNotRestrictedToOldSampledPoses)
{
    verify_free_line_exit();
}

TEST_F(PlaneTrajectoryTest, PositionAloneCannotCompleteJoin)
{
    verify_terminal_course_required();
}

TEST_F(PlaneTrajectoryTest, DoNotRestartManeuverAfterEntry)
{
    verify_entry_replan_window();
}

TEST_F(PlaneTrajectoryTest, CaptureOutgoingLineBeyondEndpoint)
{
    verify_terminal_line_capture();
}

TEST_F(PlaneTrajectoryTest, ProtectedFinishLineProvidesSettlingSpace)
{
    verify_protected_finish_line_space();
}

TEST_F(PlaneTrajectoryTest, ExcessTurningIsPenalizedWithoutRejection)
{
    verify_excess_turn_cost();
}

TEST_F(PlaneTrajectoryTest, OutgoingBoundaryDoesNotCompleteAsCorner)
{
    verify_exit_boundary();
}

TEST_F(PlaneTrajectoryTest, AcuteTurnConnectsLinesOutsideOldAcceptanceCircle)
{
    verify_acute_line_transition();
}

TEST_F(PlaneTrajectoryTest, InvalidInputsAndUnachievableWind)
{
    const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
    AP_PlaneTrajectory planner;
    EXPECT_FALSE(planner.plan(points, 0, 2, Vector2f(0, 0), 20, 45));
    EXPECT_FALSE(planner.plan(points, 1, 2, Vector2f(0, 0), NAN, 45));
    EXPECT_FALSE(planner.plan(points, 1, 2, Vector2f(0, 19), 20, 45));
    EXPECT_FALSE(planner.active());
}

TEST_F(PlaneTrajectoryTest, NoGuidanceBeforeEntryAndAfterReset)
{
    const Location points[]{location(-400, 0), location(0, 0), location(0, 400)};
    AP_PlaneTrajectory planner;
    ASSERT_TRUE(planner.plan(points, 1, 2, Vector2f(0, 0), 20, 45));
    bool complete = false;
    float bank = 0;
    EXPECT_FALSE(planner.update(points[0], Vector2f(20, 0), 2, bank, complete));
    EXPECT_TRUE(planner.planned());
    EXPECT_FALSE(planner.active());
    planner.reset();
    EXPECT_FALSE(planner.update(points[1], Vector2f(20, 0), 2, bank, complete));
}

#endif // AP_PLANE_TRAJECTORY_ENABLED

AP_GTEST_MAIN()

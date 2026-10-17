#include <AP_gtest.h>
#include <AP_CustomControl/AP_CustomControl_MIMO.h>
#include <AP_CustomControl/AP_CustomControl_LQI.h>
#include <AP_Logger/AP_Logger.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_Logger logger;
using namespace AP_CustomControl_MIMO;

TEST(CustomMIMO, CoupledTravel)
{
    const Authority authority{1, 1, 0, 1, 100, 100};
    for (int c = -2; c <= 12; c++) {
        for (int d = -12; d <= 12; d++) {
            Allocation out{};
            ASSERT_TRUE(allocate({d*0.1f, c*0.1f, 0.5f}, {}, authority, 1, out));
            EXPECT_GE(out.applied.common_brake, 0);
            EXPECT_LE(out.applied.common_brake + fabsf(out.applied.differential_brake), 1.000001f);
            EXPECT_NEAR(out.applied.differential_brake,
                        constrain_float(d*0.1f, out.applied.common_brake-1, 1-out.applied.common_brake), 1e-6f);
        }
    }
}

TEST(CustomMIMO, ActuatorSlewAndReducedAuthority)
{
    Authority authority{1, 1, 0, 1, 1, 0.5f};
    Allocation out{};
    ASSERT_TRUE(allocate({1, 0, 1}, {-1, 0, 0}, authority, 0.1f, out));
    EXPECT_NEAR(out.applied.common_brake, 0.1f, 1e-6f);
    EXPECT_NEAR(out.applied.differential_brake, -0.8f, 1e-6f);
    EXPECT_NEAR(out.applied.throttle, 0.05f, 1e-6f);
    EXPECT_TRUE(out.limits & BRAKE_SLEW);
    EXPECT_TRUE(out.limits & THROTTLE_SLEW);
    authority.common_brake_max = 0.2f;
    authority.differential_brake_max = 0.3f;
    authority.throttle_max = 0.4f;
    ASSERT_TRUE(allocate({1, 1, 1}, {0, 1, 1}, authority, 0.01f, out));
    EXPECT_LE(out.applied.common_brake, 0.2f);
    EXPECT_LE(fabsf(out.applied.differential_brake), 0.3f);
    EXPECT_LE(out.applied.throttle, 0.4f);
}

TEST(CustomMIMO, RejectInvalidVector)
{
    const Authority authority{1, 1, 0, 1, 1, 1};
    Allocation out{};
    EXPECT_FALSE(allocate({NAN, 0, 0}, {}, authority, 0.02f, out));
    EXPECT_FALSE(allocate({0, NAN, 0}, {}, authority, 0.02f, out));
    EXPECT_FALSE(allocate({0, 0, NAN}, {}, authority, 0.02f, out));
    EXPECT_FALSE(allocate({}, {}, authority, 0, out));
}

#if AP_PLANE_CUSTOMCONTROL_LQI_ENABLED
TEST(CustomMIMO, EngagementAndEstimatorReset)
{
    AP_FixedWing parameters{};
    AP_CustomControl frontend(parameters);
    AP_CustomControl_LQI controller(frontend, 0.005f);
    Input in{};
    in.time_ms = 1000;
    in.dt = 0.005f;
    in.eligible = true;
    in.reference.time_ms = in.time_ms;
    in.reference.valid = true;
    in.reference.height_home_m = 30;
    in.reference.equivalent_airspeed_mps = 4.2f;
    in.state.time_ms = in.time_ms;
    in.state.attitude_valid = in.state.navigation_valid = true;
    in.state.wind_valid = in.state.airspeed_valid = true;
    in.state.attitude_body_to_ned.from_euler(0, 0.15f, 0);
    in.state.height_home_m = 30;
    in.state.velocity_ned_mps = {4.2f, 0, 0};
    in.state.true_airspeed_mps = 4.2f;
    in.state.eas_to_tas = 1;
    in.previous.applied = {0.1f, 0.05f, 0.45f};
    ASSERT_TRUE(controller.engage(in));
    Demand output{};
    ASSERT_TRUE(controller.update_mimo(in, output));
    EXPECT_FLOAT_EQ(output.throttle, 0.45f);
    EXPECT_FLOAT_EQ(output.common_brake, 0.05f);
    EXPECT_FLOAT_EQ(output.differential_brake, 0.1f);
    for (uint8_t i = 0; i < 20; i++) {
        ASSERT_TRUE(controller.update_mimo(in, output));
        EXPECT_TRUE(output.finite());
    }
    in.state.reset_sequence++;
    EXPECT_FALSE(controller.update_mimo(in, output));
    ASSERT_TRUE(controller.engage(in));
    in.state.navigation_valid = false;
    EXPECT_FALSE(controller.update_mimo(in, output));
    EXPECT_FALSE(controller.engage(in));
    in.state.navigation_valid = true;
    in.reference.time_ms = 0;
    EXPECT_FALSE(controller.engage(in));
}
#endif

AP_GTEST_MAIN()

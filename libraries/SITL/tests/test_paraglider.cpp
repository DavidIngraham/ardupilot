#include <AP_gtest.h>
#include <SITL/SIM_Paraglider.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace SITL {

class ParagliderTest : public ::testing::Test {
protected:
    Paraglider aircraft{"paraglider"};

    Vector3f yaw_damping_moment(float speed, float yaw_rate)
    {
        aircraft.air_density = 1.225f;
        aircraft.gyro = Vector3f{0, 0, yaw_rate};
        Paraglider::ForceBreakdown forces{};
        forces.V_pf = speed;
        return aircraft.compute_torque_bf(0, 0, forces);
    }

    Vector3f thrust_moment(float throttle, float torque_per_thrust = -0.02f)
    {
        aircraft.model.prop_torque_per_thrust_m = torque_per_thrust;
        aircraft.velocity_air_bf.zero();
        aircraft.gyro.zero();
        aircraft.air_density = 1.225f;
        const auto forces = aircraft.compute_forces_bf(0, 0, throttle);
        return aircraft.compute_torque_bf(0, 0, forces);
    }

    Vector3f brake_force(float speed, float left, float right)
    {
        aircraft.velocity_air_bf = Vector3f{speed, 0, 0};
        aircraft.gyro.zero();
        aircraft.model.canopy_pitch_rad = 0;
        aircraft.air_density = 1.225f;
        return aircraft.compute_forces_bf(left, right, 0).F_brake_bf;
    }

    Vector3f brake_moment(float speed, float span, float left, float right)
    {
        aircraft.model.b_span_m = span;
        const auto forces = aircraft.compute_forces_bf(left, right, 0);
        auto differential = forces;
        differential.V_pf = speed;
        // Subtract the torque at zero brake with identical forces and state:
        // this isolates the direct asymmetric moment from force lever arms.
        return aircraft.compute_torque_bf(left, right, differential) -
               aircraft.compute_torque_bf(0, 0, differential);
    }
};

TEST_F(ParagliderTest, BrakeForceUsesDynamicPressure)
{
    const auto slow = brake_force(5, 0.3f, 0);
    const auto fast = brake_force(10, 0.3f, 0);
    EXPECT_LT(slow.x, 0); // drag opposes forward flight
    EXPECT_LT(slow.z, 0); // lift is upward in body z-down coordinates
    EXPECT_NEAR(fast.x, 4 * slow.x, 1.0e-6f);
    EXPECT_NEAR(fast.z, 4 * slow.z, 1.0e-6f);
    // Independent force magnitude from Eq. 20-21 at zero alpha.
    const float qS = 0.5f * 1.225f * 25 * 1.16f;
    EXPECT_NEAR(slow.x, -qS * 0.0001f * 0.3f, 1.0e-6f);
    EXPECT_NEAR(slow.z, -qS * 0.0021f * 0.3f, 1.0e-6f);
}

TEST_F(ParagliderTest, BrakeForceIsMirrorSymmetric)
{
    const auto left = brake_force(5, 0.3f, 0);
    const auto right = brake_force(5, 0, 0.3f);
    EXPECT_NEAR(left.x, right.x, 1.0e-6f);
    EXPECT_NEAR(left.z, right.z, 1.0e-6f);
    EXPECT_NEAR(left.y, 0, 1.0e-6f);
    EXPECT_NEAR(brake_force(5, 0, 0).length(), 0, 1.0e-6f);
    EXPECT_NEAR(brake_force(0, 0.3f, 0).length(), 0, 1.0e-6f);
}

TEST_F(ParagliderTest, BrakeMomentsUseSpanSquared)
{
    brake_force(5, 0, 0);
    const auto moment = brake_moment(5, 2.15f, 0, 0.3f);
    const float scale = 0.5f * 1.225f * 25 * 1.16f * sq(2.15f) / 0.4f * 0.15f;
    EXPECT_NEAR(moment.x, scale * 0.0001f, 1.0e-6f);
    EXPECT_NEAR(moment.z, scale * 0.004f, 1.0e-6f);
    const auto wider = brake_moment(5, 4.3f, 0, 0.3f);
    EXPECT_NEAR(wider.x, 4 * moment.x, 1.0e-6f);
    EXPECT_NEAR(wider.z, 4 * moment.z, 1.0e-6f);
    const auto faster = brake_moment(10, 2.15f, 0, 0.3f);
    EXPECT_NEAR(faster.z, 4 * moment.z, 1.0e-6f);
}

TEST_F(ParagliderTest, BrakeMomentsReverseAndCancel)
{
    brake_force(5, 0, 0);
    const auto left = brake_moment(5, 2.15f, 0.3f, 0);
    const auto right = brake_moment(5, 2.15f, 0, 0.3f);
    EXPECT_LT(left.z, 0);
    EXPECT_GT(right.z, 0);
    EXPECT_NEAR(left.x, -right.x, 1.0e-6f);
    EXPECT_NEAR(left.z, -right.z, 1.0e-6f);
    EXPECT_NEAR(brake_moment(5, 2.15f, 0.3f, 0.3f).length(), 0, 1.0e-6f);
}

TEST_F(ParagliderTest, YawDampingOpposesRotationAndScalesWithSpeed)
{
    const auto positive = yaw_damping_moment(5, 0.2f);
    const auto negative = yaw_damping_moment(5, -0.2f);
    const float expected = -0.05f * 0.5f * 1.225f * 5 * 1.16f * sq(2.15f) * 0.2f / 2;
    EXPECT_NEAR(positive.z, expected, 1.0e-6f);
    EXPECT_LT(positive.z * 0.2f, 0); // damping removes rotational energy
    EXPECT_LT(negative.z * -0.2f, 0);
    EXPECT_NEAR(negative.z, -positive.z, 1.0e-6f);
    EXPECT_NEAR(yaw_damping_moment(10, 0.2f).z, 2 * positive.z, 1.0e-6f);
    EXPECT_NEAR(yaw_damping_moment(5, 0.4f).z, 2 * positive.z, 1.0e-6f);
    EXPECT_NEAR(yaw_damping_moment(5, 0).z, 0, 1.0e-6f);
    EXPECT_NEAR(yaw_damping_moment(0, 0.2f).z, 0, 1.0e-6f);
}

TEST_F(ParagliderTest, PropellerReactionTorqueTracksThrustAndRotationDirection)
{
    const auto full = thrust_moment(1);
    EXPECT_NEAR(full.x, -0.2f, 1.0e-6f); // 10 N * -0.02 m
    EXPECT_NEAR(full.y, 1.37f, 1.0e-6f); // existing thrust-offset moment
    EXPECT_NEAR(full.z, 0, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(0.4f).x, 0.4f * full.x, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(0).length(), 0, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(-1).length(), 0, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(2).x, full.x, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(1, 0).x, 0, 1.0e-6f);
    EXPECT_NEAR(thrust_moment(1, 0.02f).x, -full.x, 1.0e-6f);
}

} // namespace SITL

AP_GTEST_MAIN()

#include <AP_gtest.h>
#include <APM_Control/PG_TurnController.h>
#include <AP_Logger/AP_Logger.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_Logger logger;

#if AP_TECS_PARAGLIDER_ENABLED
TEST(ParagliderControl, ReferenceAndAllocation)
{
    EXPECT_FLOAT_EQ(PG_TurnController::heading_rate_target(0, 1, 5, 1), 0);
    const float target = PG_TurnController::heading_rate_target(radians(20), 1, 5, 1);
    EXPECT_NEAR(target, degrees(GRAVITY_MSS * tanf(radians(20)) / 5), 1.0e-5f);
    EXPECT_FLOAT_EQ(PG_TurnController::heading_rate_target(radians(20), 1, 5, -1),
                    PG_TurnController::heading_rate_target(radians(20), 1, 5, 0.3f));
    for (int common = 0; common <= 100; common += 10) {
        for (int diff = -150; diff <= 150; diff += 10) {
            const Vector2f out = PG_TurnController::allocate_brakes(diff, common);
            EXPECT_FLOAT_EQ(MIN(out.x, out.y), common);
            EXPECT_LE(MAX(out.x, out.y), 100);
            EXPECT_NEAR(out.y - out.x, constrain_float(diff, common - 100, 100 - common), 1.0e-5f);
        }
    }
}

TEST(ParagliderControl, ReturnMatchesOutputAndReleasesResidual)
{
    PG_TurnController controller;
    controller.reset(2700);
    EXPECT_NEAR(controller.update(0, 0, 0, 5, 0.02f), 2700, 0.001f);
    float previous = 2700;
    for (unsigned i = 0; i < 200; i++) {
        const float output = controller.update(0, 0, 0, 5, 0.02f);
        EXPECT_LE(output, previous + 0.001f);
        EXPECT_GE(output, 0);
        previous = output;
    }
    EXPECT_LT(previous, 700); // IMAX is retained, the excess handover demand decays.
    controller.reset(-4500);
    EXPECT_NEAR(controller.update(0, 0, 0, 5, 0.02f, 0.2f), -900, 0.001f);
    controller.reset();
    EXPECT_FLOAT_EQ(controller.update(0, 0, 0, 5, 0.02f), 0);
}
#endif

AP_GTEST_MAIN()

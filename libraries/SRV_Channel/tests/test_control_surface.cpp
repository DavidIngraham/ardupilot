#include <AP_gtest.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(SRV_Channel, ParagliderBrakesAreControlSurfaces)
{
    EXPECT_TRUE(SRV_Channel::is_control_surface(SRV_Channel::k_pg_brake_left));
    EXPECT_TRUE(SRV_Channel::is_control_surface(SRV_Channel::k_pg_brake_right));
}

TEST(SRV_Channel, ExistingControlSurfaceClassification)
{
    EXPECT_TRUE(SRV_Channel::is_control_surface(SRV_Channel::k_aileron));
    EXPECT_TRUE(SRV_Channel::is_control_surface(SRV_Channel::k_airbrake));
    EXPECT_FALSE(SRV_Channel::is_control_surface(SRV_Channel::k_throttle));
    EXPECT_FALSE(SRV_Channel::is_control_surface(SRV_Channel::k_none));
}

AP_GTEST_MAIN()

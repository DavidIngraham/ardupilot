#include "AP_AngleSensor_config.h"

#if AP_ANGLESENSOR_ENABLED

#include "AP_AngleSensor.h"
#include "AP_AngleSensor_Params.h"


// table of user settable parameters
const AP_Param::GroupInfo AP_AngleSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Angle Sensor type
    // @Description: What type of Angle Sensor is connected
    // @Values: 0:None,1:AS5048B
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_AngleSensor_Params, _type, TYPE::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: OFFS
    // @DisplayName: Zero Position Offset, in Degrees
    // @Description: This offset is added to the measured angle
    // @Units: deg
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("OFFS",    4, AP_AngleSensor_Params, _offset, ANGLE_SENSOR_DEFAULT_OFFSET),

    // @Param: DIR
    // @DisplayName: Axis Direction
    // @Description: Axis Direction, set to -1 to reverse
    // @Values: -1: Reversed, 1: Default
    // @User: Standard
    AP_GROUPINFO("DIR",    5, AP_AngleSensor_Params, _direction, ANGLE_SENSOR_DIRECTION_FORWARDS),

    AP_GROUPEND
};

AP_AngleSensor_Params::AP_AngleSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif //AP_ANGLESENSOR_ENABLED

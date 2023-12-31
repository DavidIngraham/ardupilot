#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_AngleSensor_config.h"

#define LOG_IDS_FROM_ANGLESENSOR \
    LOG_ANGLESENSOR_MSG

// @LoggerMessage: AENC
// @Description: Angle Sensor Status
// @Field: TimeUS: Time since system startup
// @Field: Inst: Angle sensor instance number
// @Field: Angle: Absolute angle measurement
// @Field: Qual: Measurement quality 


struct PACKED log_AngleSensor {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float angle;
    uint8_t quality;
};

#if !AP_ANGLESENSOR_ENABLED
#define LOG_STRUCTURE_FROM_ANGLESENSOR
#else
#define LOG_STRUCTURE_FROM_ANGLESENSOR \
   { LOG_ANGLESENSOR_MSG, sizeof(log_AngleSensor), \
      "ANG",  "QBfbfb", "TimeUS,InstAngle0,Qual0,Angle1,Qual1", "s#r%r%", "F-0-0-" , true }, 
#endif

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_FuelMonitor_Backend.h"

class AP_FuelMonitor_Analog_Level : public AP_FuelMonitor_Backend
{
public:
    AP_FuelMonitor_Analog_Level(AP_FuelMonitor &frontend);

    // probe and initialise the sensor
    bool init(void) override;

    // return the current volume reading
    bool get_volume_measurement(float &volume) override;

    // This type of sensor cannot provide a rate 
    bool get_rate(float &rate) override 
    {
    	return false;
    }

private:
    AP_HAL::AnalogSource *_source;

};

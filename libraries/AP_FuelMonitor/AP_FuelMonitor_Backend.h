#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_FuelMonitor.h"

class AP_FuelMonitor;

class AP_FuelMonitor_Backend
{
public:
  friend class AP_FuelMonitor; 

  // constructor. This incorporates initialisation as well.
  AP_FuelMonitor_Backend(AP_FuelMonitor &frontend);

  // we declare a virtual destructor so that fuel monitor driver can
    // override with a custom destructor if need be
  virtual ~AP_FuelMonitor_Backend(void);

  virtual bool init(void) = 0;

  // Pass by reference because not all backends support each measurement
  virtual bool get_volume_measurement(float &volume) = 0;

  virtual bool get_rate(float &rate) = 0;


protected:

  int8_t get_pin(void) const;
  int8_t get_ratio(void) const;
  int8_t get_offset(void) const;


private:

    AP_FuelMonitor &frontend;
    
};

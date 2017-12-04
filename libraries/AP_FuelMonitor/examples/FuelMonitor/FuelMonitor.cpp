/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   FuelMonitor.cpp - fuel monitor example sketch
 *
 */

#include <AP_ADC/AP_ADC.h>
#include <AP_FuelMonitor/AP_FuelMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

float temperature;

AP_FuelMonitor::vehicle_status_t status;

AP_FuelMonitor fuelmonitor;
static AP_BoardConfig board_config = AP_BoardConfig::create();

namespace {
  // try to set the object value but provide diagnostic if it failed
  void set_object_value(const void *object_pointer,
                        const struct AP_Param::GroupInfo *group_info,
                        const char *name, float value)
  {
      if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
          hal.console->printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
                              group_info->name, name);
      }
  }
}

void setup()
{
    hal.console->printf("ArduPilot Fuel Monitor library test\n");

    set_object_value(&fuelmonitor, fuelmonitor.var_info, "TYPE", 1);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "OFFSET", 0);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "RATIO", 20);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "PIN", 1);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "CAPACITY", 1);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "BURN_RATE_IDLE", 0);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "BURN_RATE_CRUISE", 0);
    set_object_value(&fuelmonitor, fuelmonitor.var_info, "ANGLE_MAX", 5);

    board_config.init();

    fuelmonitor.init();
}

void loop(void)
{
    static uint32_t timer;
    if ((AP_HAL::millis() - timer) > 100) {
        timer = AP_HAL::millis();

        status.throttle_percent = 50;
        status.throttle_percent_cruise = 50;
        status.engine_armed = 1;
        status.roll = 0;
        status.pitch = 0;


        fuelmonitor.read(status);

        hal.console->printf("Percent %5.2f mass %6.2f\n",
                            (double)fuelmonitor.get_percent(), (double)fuelmonitor.get_mass_remaining());
    }
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();

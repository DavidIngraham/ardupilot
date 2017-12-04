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
 *   analog fuel level sensor driver
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_FuelMonitor.h"
#include "AP_FuelMonitor_Analog_Level.h"

extern const AP_HAL::HAL &hal;


AP_FuelMonitor_Analog_Level::AP_FuelMonitor_Analog_Level(AP_FuelMonitor &_frontend) :
    AP_FuelMonitor_Backend(_frontend)
{
    _source = hal.analogin->channel(get_pin());
}

bool AP_FuelMonitor_Analog_Level::init()
{
    return _source != nullptr;
}

// read the level sensor
bool AP_FuelMonitor_Analog_Level::get_volume_measurement(float &volume)
{
    if (_source == nullptr) {
        return false;
    }
    // allow pin to change
    _source->set_pin(get_pin());

    // Scale the level reading to an approximate volume
    volume = (_source->voltage_average() - get_offset()) * get_ratio() ;

    return true;
}


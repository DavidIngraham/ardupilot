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
  backend driver class for fuel sensor
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_FuelMonitor.h"

extern const AP_HAL::HAL &hal;

AP_FuelMonitor_Backend::AP_FuelMonitor_Backend(AP_FuelMonitor &_frontend) :
    frontend(_frontend)
{
    // nothing to do here
}

AP_FuelMonitor_Backend::~AP_FuelMonitor_Backend(void)
{
    // nothing to do here
}

int8_t AP_FuelMonitor_Backend::get_pin(void) const
{
    return frontend._pin;
}

int8_t AP_FuelMonitor_Backend::get_ratio(void) const
{
    return frontend._ratio;
}

int8_t AP_FuelMonitor_Backend::get_offset(void) const
{
    return frontend._offset;
}


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



#include "AP_AngleSensor_I2C_AS5048B.h"

#if AP_ANGLESENSOR_ENABLED

const AP_Param::GroupInfo AP_AngleSensor_I2C::var_info[] = {

    // Param indexes must be between 10 and 19 to avoid conflict with other battery monitor param tables loaded by pointer

    /// @Param: BUS
    // @DisplayName: Angle Sensor Serial Bus Index
    // @Description: Angle Sensor Serial Bus Index
    // @Values: 1: I2C1, 2: I2C2, 3: I2C3
    // @User: Standard
    AP_GROUPINFO("I2C_BUS",    2, AP_AngleSensor_Params, _bus, ANGLE_SENSOR_BUS_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Serial Bus Address
    // @Description: Serial Bus Address
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("I2C_ADDR",    3, AP_AngleSensor_Params, _addr, ANGLE_SENSOR_ADDR_DEFAULT),

    // Param indexes must be between 10 and 19 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

#endif  // AP_ANGLESENSOR_ENABLED

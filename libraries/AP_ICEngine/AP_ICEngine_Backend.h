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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"

class AP_ICEngine_Backend
{
public:
   // constructor. This incorporates initialisation as well.
   AP_ICEngine_Backend(AP_ICEngine_Backend &frontend, AP_ICEngine::ICEngine_State &ice_state, AP_ICEngine::Params &params);

   // we declare a virtual destructor so that ICEngine driver can
   // override with a custom destructor if need be
   virtual ~AP_ICEngine_Backend(void) {}

   // initialise
   virtual void init() = 0;

   // read the latest engine data
   virtual void read() = 0;

   /// returns true if the engine backend includes starter control
   virtual bool has_start_control() const = 0;


protected:
   AP_ICEngine                     &_frontend;      // reference to front-end
   AP_ICEngine::ICEngine_State     &_state;    // reference to this instances state (held in the front-end)
   AP_ICEngine_Params              &_params;   // reference to this instances parameters (held in the front-end)

private:
   //TBD
};

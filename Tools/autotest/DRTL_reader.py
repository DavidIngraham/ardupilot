#!/usr/bin/env python

'''
Basic Example for testing DRTL

Reads a heartbeat and GLOBAL_POSITION_INT 

'''

from __future__ import print_function

import time
import math
from pymavlink import mavutil

SYSID_OUT = 101
COMPID_OUT = 0

lat = -35.3620798 # CMAC runway north end
lng = 149.1650704
alt = 583.96

UPDATE_PERIOD = 1 # send every second

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))



if __name__ == "__main__":

	master = mavutil.mavlink_connection('udp:127.0.0.1:14551', SYSID_OUT, COMPID_OUT)

	# wait for the heartbeat msg to find the system ID
	wait_heartbeat(master)

	last_update = 0

	counter = 0

	while True:
		if (time.time() - last_update) > UPDATE_PERIOD:

			lat += math.sin(counter/10)/100

			print(lat)

			master.mav.heartbeat_send(31,3,0,0,4,2)
			master.mav.global_position_int_send(time.time()/1000,lat, lng, alt, 0, 0, 0 , 0, 0)

			counter += 1
			last_update = time.time()



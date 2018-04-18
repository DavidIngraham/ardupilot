#!/usr/bin/env python

'''
Basic Example for testing DRTL

Publishes a heartbeat and GLOBAL_POSITION_INT 

'''

from __future__ import print_function

import time
import math
from pymavlink import mavutil

SYSID_OUT = 111

lat_origin = -35.3620798 # CMAC runway north end
lng_origin = 149.1650704
alt_origin = 583.96

UPDATE_PERIOD = 100 # milliseconds

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))



if __name__ == "__main__":

	master = mavutil.mavlink_connection('udp:127.0.0.1:14551', source_system = SYSID_OUT)

	# wait for the heartbeat msg to find the system ID
	wait_heartbeat(master)

	last_update = 0

	counter = 0

	start_time = int(time.time()*1000)

	while True:
		if (time.time()*1000 - last_update) > UPDATE_PERIOD:

			lat = lat_origin + math.sin(counter/100)/10000 # move up and down the cmac runway
			lng = lng_origin
			alt = alt_origin

			print(int(time.time()*1000))

			if (counter % (1000/UPDATE_PERIOD) == 0): # send heartbeat at 1 Hz
				master.mav.heartbeat_send(31,3,0,0,4,2)
			master.mav.global_position_int_send(int(time.time()*1000-start_time), lat*1E7, lng*1E7, alt*1000, 0, 0, 0, 0, 0)

			counter += 1
			last_update = time.time()








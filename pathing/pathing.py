from movement_func import *
from track_and_align import *

def pathing(stage, tag_info):
	# return next_stage, flight_time
	match stage:
		case 1: #stabilize 
			tello.send_rc_control(0, 0, 0, 0)
			next_stage = stage + 1, flight_time = 1
			
		case 2:

		case 3:

		case 4:

	return next_stage, flight_time


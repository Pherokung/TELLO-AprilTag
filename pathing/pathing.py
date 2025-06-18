from movement_func import *
from track_and_align import *

DEFAULT_PID_ERRORS = pid_errors = {
    'pError_yaw_track': 0,
    'pError_ud_track': 0,
    'pError_lr_pixel_pos': 0,
    'pError_ud_pixel_pos': 0
}

def track_and_align_with_tag_id(tello, stage, detected_tags, state, pid_errors, frame_siz, 
								tag_id, target_area, target_x, target_y, target_yaw):

	next_stage = stage, flight_time = 0.01
	tag = is_tag_exist(detected_tags, tag_id)
	if tag:
		upd_state, upd_pid_errors, rc_sent = master_track_and_align_apriltag(
			tello, tag, 
			frame_siz[0], frame_siz[1],
			target_area, target_x, 
			target_y, target_yaw,
			state, pid_errors,	 
		)

		if upd_state == STATE_HOLDING:
			# go to next stage and reset everything
			next_stage = stage + 1, upd_state = STATE_IDLE, upd_pid_errors = DEFAULT_PID_ERRORS
	else:
		upd_state = state, upd_pid_errors = pid_errors
	
	return next_stage, flight_time, state, pid_errors

def pathing(tello, stage, detected_tags, state, pid_errors, frame_siz):
	"""
		Pathing function for all stages of the drone path

		Args:
			tello: the tello obj
			stage: current stage of the drone pathing
			detected_tags: all tags detected by the detect_apriltag func
			state: current state of the drone
			pid_errors: updated pid_errors
			frame_siz: contain [frame_width, frame_height]

		Return:
			next_stage: updated stage
			flight_time: how long this function needs to be call again
			upd_state: updated state
			upd_pid_errors updated pid_errors
	"""

	match stage:
		case 1: #stabilize 
			tello.send_rc_control(0, 0, 0, 0)
			next_stage = stage + 1, flight_time = 1, upd_state = STATE_IDLE, upd_pid_errors = DEFAULT_PID_ERRORS
			
		case 2: #align the selected tag
			
			next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag_id(
				tello, stage, detected_tags, state, pid_errors, frame_siz, 
				tag_id = 0,
				target_area = 11000,
				target_x = 0,
				target_y = 0,
				target_yaw = 0
			)

		case 3: #moving the telo forward
			flight_time = compound_move_tello(tello, "up", 50, "forward", 150)
			next_stage = stage + 1, flight_time = 1, upd_state = STATE_IDLE, upd_pid_errors = DEFAULT_PID_ERRORS

		case 4:

			

	# return next_stage, flight_time
	return next_stage, flight_time, state, pid_errors


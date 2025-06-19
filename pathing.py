from movement_func import *
from track_and_align import *
from dataclasses import dataclass
from typing import Optional

DEFAULT_PID_ERRORS = {
    'pError_yaw_track': 0,
    'pError_ud_track': 0,
    'pError_lr_pixel_pos': 0,
    'pError_ud_pixel_pos': 0
}

@dataclass
class Target:
	id: int
	area: float
	x_offset_px: float
	y_offset_px: float
	final_yaw_rad: float

@dataclass
class Tolerance:
    area_tracking: Optional[float] = None
    final_yaw_rad: Optional[float] = None
    final_position_px: Optional[float] = None
    offset_tracking_px: Optional[float] = None

def track_and_align_with_tag(tello, stage, tag, state, pid_errors, frame_siz, target, tolerance):
	"""Helper function to perform tracking and alignment for a specific tag."""
	next_stage = stage
	flight_time = 0.01

	upd_state, upd_pid_errors, rc_sent = master_track_and_align_apriltag(
		tello, tag, 
		frame_siz[0], frame_siz[1],
		target.area, target.x_offset_px, 
		target.y_offset_px, target.final_yaw_rad,
		state, pid_errors,
		align_yaw_power=10,
		align_lr_power=10,
		area_tolerance_tracking=tolerance.area_tracking,
		final_position_tolerance_px=tolerance.final_position_px,
		final_yaw_tolerance_rad=tolerance.final_yaw_rad,
	)

	if upd_state == STATE_HOLDING:
		# Alignment complete, move to next stage
		next_stage = stage + 1
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	return next_stage, flight_time, upd_state, upd_pid_errors

def pathing(tello, stage, detected_tags, state, pid_errors, frame_siz=[960, 720]):
	"""
	Pathing function for all stages of the drone path.

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
		upd_pid_errors: updated pid_errors
	"""
	# Initialize return values to prevent UnboundLocalError
	next_stage = stage
	flight_time = 0.1  # Default flight time
	upd_state = state
	upd_pid_errors = pid_errors

	if stage == 1:  # Stabilize
		tello.send_rc_control(0, 0, 0, 0)
		next_stage = stage + 1
		flight_time = 1
		upd_state = STATE_TRACKING
		upd_pid_errors = DEFAULT_PID_ERRORS
		
	elif stage == 2:  # Align with the first set of tags
		target_1 = Target(id=1, area=956, x_offset_px=0, y_offset_px=-180, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=100, final_yaw_rad=0.1, final_position_px=70, offset_tracking_px=10)
		
		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1

			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		if not target_found:
			tello.send_rc_control(0, 0, 0, 0)
			upd_state = STATE_TRACKING
			
	elif stage == 3:  # Move the Tello forward
		flight_time = move_tello(tello, "forward", 100)
		next_stage = stage + 1
		upd_state = STATE_TRACKING
		upd_pid_errors = DEFAULT_PID_ERRORS

	elif stage == 4: # Align with the second set of tags
		target_1 = Target(id=1, area=4200, x_offset_px=-25, y_offset_px=0, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=500, final_yaw_rad=0.1, final_position_px=20, offset_tracking_px=25)

		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1
			
			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		
		if not target_found:
			tello.send_rc_control(0, 0, 0, 0)
			upd_state = STATE_TRACKING
		

	elif stage == 5: #move through the second gate
		next_stage = stage + 1
		flight_time = compound_move_tello(tello, "up", 70, "forward", 150)
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 6:  #move in a curve to pass the first flag
		curve_tello(tello, 75, -30, 0, 150, 0, 0)
		next_stage = stage + 1
		flight_time = 0.5
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
  
	elif stage == 7: #move in a curve to pass the second flag
		curve_tello(tello, 75, 30, 0, 150, 0, 0)
		next_stage = stage + 1
		flight_time = 0.5
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 8: #turn 90 degree
		next_stage = stage + 1
		flight_time = turn_tello(tello, 90, speed=50)
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 9: #stop the drone's movement
		tello.send_rc_control(0, 0, 0, 0)
		next_stage = stage + 1
		flight_time = 0.2
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 10: #move the drone to infront of the tag
		flight_time = compound_move_tello(tello, "down", 30, "left", 50)
		next_stage = stage + 1
		upd_state = STATE_TRACKING #Must BE STATE_TRACKING if the next stage in aligning!!!
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	# =================================================================== #
	# AFTER THIS PART OF THE CODE, ALL VALUES NEED TO BE TUNED AND TESTED #
	# =================================================================== #
 
	elif stage == 12: #aligning with the tag
		target_1 = Target(id=1, area=4200, x_offset_px=-25, y_offset_px=0, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=500, final_yaw_rad=0.1, final_position_px=20, offset_tracking_px=25)

		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1
			
			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		
		if not target_found:
			tello.send_rc_control(0, 0, 0, 0)
			upd_state = STATE_TRACKING
	
	elif stage == 13:  # The drone move through the tunnel
		flight_time = move_tello(tello, "forward", 100)
		next_stage = stage + 1
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 14: # Turn 90 degree
		next_stage = stage + 1
		flight_time = turn_tello(tello, 90, speed=50)
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
  
	elif stage == 15: # Move the drone to infront of the next tag
		flight_time = compound_move_tello(tello, "down", 30, "left", 50)
		next_stage = stage + 1
		upd_state = STATE_TRACKING #Must BE STATE_TRACKING if the next stage in aligning!!!
		upd_pid_errors = DEFAULT_PID_ERRORS
	
	elif stage == 16: #aligning with the tag
		target_1 = Target(id=1, area=4200, x_offset_px=-25, y_offset_px=0, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=500, final_yaw_rad=0.1, final_position_px=20, offset_tracking_px=25)

		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1
			
			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		
		if not target_found:
			tello.send_rc_control(0, 0, 0, 0)
			upd_state = STATE_TRACKING
	
	
			
	return next_stage, flight_time, upd_state, upd_pid_errors


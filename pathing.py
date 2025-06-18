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
		target_2 = Target(id=5, area=991, x_offset_px=105.8, y_offset_px=-335, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=100, final_yaw_rad=0.1, final_position_px=70, offset_tracking_px=20)
		
		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1
			elif tag['tag_id'] == target_2.id:
				current_target = target_2

			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		
		if not target_found:
			# If no target tag is found, hover and search
			upd_state = STATE_IDLE
			upd_pid_errors = DEFAULT_PID_ERRORS
			
	elif stage == 3:  # Move the Tello forward
		move_tello(tello, "forward", 20)
		next_stage = stage + 1
		flight_time = 1
		upd_state = STATE_TRACKING
		upd_pid_errors = DEFAULT_PID_ERRORS

	elif stage == 4: # Align with the second set of tags
		target_1 = Target(id=1, area=5137, x_offset_px=-25, y_offset_px=284, final_yaw_rad=0)
		target_2 = Target(id=5, area=5594, x_offset_px=351, y_offset_px=-213, final_yaw_rad=0)
		tolerance = Tolerance(area_tracking=100, final_yaw_rad=0.1, final_position_px=20, offset_tracking_px=20)

		target_found = False
		for tag in detected_tags:
			current_target = None
			if tag['tag_id'] == target_1.id:
				current_target = target_1
			elif tag['tag_id'] == target_2.id:
				current_target = target_2
			
			if current_target:
				next_stage, flight_time, upd_state, upd_pid_errors = track_and_align_with_tag(
					tello, stage, tag, state, pid_errors, frame_siz, current_target, tolerance
				)
				target_found = True
				break
		
		if not target_found:
			# If no target tag is found, hover and search
			upd_state = STATE_IDLE
			upd_pid_errors = DEFAULT_PID_ERRORS

	elif stage == 5:
		compound_move_tello(tello, "up", 50, "forward", 50)
		next_stage = stage + 1
		flight_time = 1
		upd_state = STATE_IDLE
		upd_pid_errors = DEFAULT_PID_ERRORS
			
	return next_stage, flight_time, upd_state, upd_pid_errors


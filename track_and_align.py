import numpy as np

AREA_TOLERANCE_TRACKING = 300  # Renamed for clarity, used for tracking area check
PIXEL_OFFSET_TOLERANCE_TRACKING = 20 # Tolerance for pixel offset from center during tracking
POSITIONING_PIXEL_TOLERANCE = 10 # Tolerance for pixel-based positioning

# PID Controller Gains Explanation:
# Each list below represents [Kp, Ki, Kd]
# - Kp (Proportional Gain): 
#   - Determines the reaction to the current error. 
#   - A higher Kp makes the drone react more aggressively to errors.
#   - Too high: Can lead to overshoot and oscillation (shaking or wobbling).
#   - Too low: Drone will be sluggish and may not reach the target effectively.
# - Ki (Integral Gain):
#   - Corrects steady-state errors (errors that persist over time).
#   - Accumulates past errors. If the drone is consistently off-target, Ki helps to eliminate this offset.
#   - Too high: Can cause overshoot, oscillations, or instability (wind-up).
#   - Too low (or 0): May result in the drone not quite reaching the target if there are persistent disturbances (like drift).
#   - Often set to 0 initially and tuned only if necessary.
# - Kd (Derivative Gain):
#   - Predicts future errors based on the current rate of change of the error.
#   - Dampens oscillations and reduces overshoot by slowing down the drone as it approaches the target.
#   - Too high: Can make the drone overly sensitive to noise, leading to jerky movements.
#   - Too low: May not effectively dampen oscillations.
#   - Often set to 0 initially and tuned to improve stability and reduce overshoot.
#
# Tuning Process Generally:
# 1. Start with Ki = 0 and Kd = 0.
# 2. Increase Kp until the drone responds quickly to errors. If it oscillates, reduce Kp.
# 3. If there's a persistent error (drone stops short of target), slowly increase Ki.
# 4. If there's overshoot or oscillation, slowly increase Kd to dampen it.
# Repeat tuning for each axis/control loop (yaw, up/down, left/right) independently if possible.

pid_yaw = [0.4, 0.4, 0]  # For yaw control (rotation based on pixel offset during tracking, or angle during alignment)
pid_ud = [0.30, 0.15, 0]   # For up/down control (vertical translation based on pixel offset or direct y-coordinate error)
pid_lr = [0.20, 0.15, 0]   # For left/right control (lateral translation based on pixel offset or direct x-coordinate error)
pid_lr_yaw = [0.15, 0.15, 0] 

def is_aligning_done(current_angle_yaw_rad, desired_angle_rad, angle_threshold_rad):
    """
    Checks if the drone's current yaw angle is within the threshold of the desired angle.
    current_angle_yaw_rad: The current yaw of the drone relative to the tag (e.g., angle[1] from tag_info), in radians.
    desired_angle_rad: The target yaw angle, in radians.
    angle_threshold_rad: The allowed tolerance, in radians.
    """
    error_angle_rad = current_angle_yaw_rad - desired_angle_rad
    # Normalize error to be between -pi and pi if necessary, though for small differences it might not be critical
    # error_angle_rad = (error_angle_rad + np.pi) % (2 * np.pi) - np.pi 
    if abs(error_angle_rad) < angle_threshold_rad:
        return True
    return False # Explicitly return False

def aligning_angle(tello, current_angle_yaw_rad, desired_angle_rad, angle_threshold_rad, yaw_power=30, lr_power=25):
    """
    Adjusts the Tello's yaw to meet the desired_angle_rad relative to the tag.
    current_angle_yaw_rad: The current yaw (angle[1] from tag_info), in radians.
    desired_angle_rad: The target yaw angle, in radians.
    angle_threshold_rad: The allowed tolerance for completion, in radians.
    yaw_power: The base speed for yaw correction (10-100).
    """

    if is_aligning_done(current_angle_yaw_rad, desired_angle_rad, angle_threshold_rad):
        tello.send_rc_control(0, 0, 0, 0)
        # print(f"Aligned! Current Angle: {current_angle_yaw_rad:.2f} rad, Desired: {desired_angle_rad:.2f} rad")
        return True
    else:
        error_angle_rad = current_angle_yaw_rad - desired_angle_rad
        
        # Determine yaw_speed based on the error
        # If error_angle_rad is positive, current > desired, so drone needs to yaw LEFT (negative Tello yaw speed)
        # If error_angle_rad is negative, current < desired, so drone needs to yaw RIGHT (positive Tello yaw speed)
        
        # Simple proportional control for yaw speed, can be made more sophisticated (PID)
        # For Tello, positive yaw is clockwise (RIGHT), negative is counter-clockwise (LEFT).
        
        yaw_speed_command = 0
        # Re-introducing lr_speed_command for combined yaw and lateral movement during alignment.
        lr_speed_command = 0 

        if error_angle_rad < -angle_threshold_rad: # Current angle is too far to the right of desired, need to yaw LEFT
            yaw_speed_command = -int(yaw_power) 
            lr_speed_command = int(lr_power) # Move RIGHT while yawing LEFT
        elif error_angle_rad > angle_threshold_rad: # Current angle is too far to the left of desired, need to yaw RIGHT
            yaw_speed_command = int(yaw_power)
            lr_speed_command = -int(lr_power) # Move LEFT while yawing RIGHT
        else:
            # Within deadband but not yet meeting is_aligning_done (should be rare if thresholds match)
            yaw_speed_command = 0
            lr_speed_command = 0
        
        tello.send_rc_control(lr_speed_command, 0, 0, yaw_speed_command)
        # print(f"Aligning... Current: {current_angle_yaw_rad:.2f}, Desired: {desired_angle_rad:.2f}, Error: {error_angle_rad:.2f}, YawCmd: {yaw_speed_command}, LRCmd: {lr_speed_command}")
        return False
    
def is_tracking_done(current_area, target_area, current_offset_x, current_offset_y, 
                       area_tolerance, offset_tolerance_px): # Added tolerance parameters
    """
    Checks if tracking is done based on area and pixel offsets from center.
    current_area: detected area of the tag.
    target_area: desired area of the tag.
    current_offset_x: horizontal pixel offset of tag center from frame center.
    current_offset_y: vertical pixel offset of tag center from frame center.
    area_tolerance: allowed difference from target_area.
    offset_tolerance_px: allowed pixel offset from center (for x and y).
    """
    area_condition = abs(current_area - target_area) < area_tolerance # Use parameter
    # offset_x_condition = abs(current_offset_x) < offset_tolerance_px # Use parameter
    # offset_y_condition = abs(current_offset_y) < offset_tolerance_px # Use parameter

    if area_condition:
        # print(f"Tracking done: Area diff {abs(current_area - target_area)}, Offset X {abs(current_offset_x)}, Offset Y {abs(current_offset_y)}")
        return True
    # print(f"Tracking not done: Area diff {abs(current_area - target_area)} (Area: {current_area} vs Target: {target_area}), Offset X {abs(current_offset_x)}, Offset Y {abs(current_offset_y)}")
    return False
      

def track_apriltag(tello, area, 
                   ox, oy,  # ox: horizontal pixel offset, oy: vertical pixel offset
                   pError_yaw, pError_ud_track, # pError_ud_track to distinguish from positioning
                   target_area, area_tolerance):

    error_yaw_pixel = ox  # Current horizontal pixel offset
    # Yaw speed control based on horizontal pixel offset
    yaw_speed = pid_yaw[0] * error_yaw_pixel + pid_yaw[1] * (error_yaw_pixel - pError_yaw)
    yaw_speed = int(np.clip(yaw_speed, -100, 100))

    # Forward/backward speed control based on area
    fb_speed = 0
    if area > target_area + area_tolerance:  # Too close
        fb_speed = -20  # Move back
    elif area < target_area - area_tolerance and area != 0:  # Too far
        fb_speed = 20  # Move forward
    
    # Up/down speed control based on vertical pixel offset
    # Tello positive RC 'ud' is UP.
    # If oy is positive (tag below center), we need to move UP (positive ud command).
    # So, error_ud_pixel_input should be positive if tag is below center.
    # However, the original code used error_ud = -oy.
    # If oy = (center_y - frame_height // 2) is positive (tag below center), then -oy is negative.
    # pid_ud[0] * (-oy) would result in a negative 'ud' command (DOWN). This is correct to reduce 'oy'.
    error_ud_pixel_input = -oy # If oy > 0 (tag below center), error_ud_pixel_input < 0.
    ud_speed = pid_ud[0] * error_ud_pixel_input + pid_ud[1] * (error_ud_pixel_input - pError_ud_track)
    ud_speed = int(np.clip(ud_speed, -100, 100)) # If error_ud_pixel_input < 0, ud_speed < 0 (DOWN). Correct.

    tello.send_rc_control(0, fb_speed, ud_speed, yaw_speed) # lr, fb, ud, yaw
    # print("----------------")
    # print(f"Tracking: Area({area:.0f}/{target_area:.0f}), Offsets(px,py):({ox:.0f},{oy:.0f})")
    # print(f"Tracking RC: FB({fb_speed}), UD({ud_speed}), Yaw({yaw_speed})")

    return error_yaw_pixel, error_ud_pixel_input # Return current pixel errors for PD controller

def is_positioning_pixel_done(current_tag_center_x_coord, current_tag_center_y_coord,
                               current_area,
                               frame_width, frame_height,
                               target_x_offset_from_frame_center, target_y_offset_from_frame_center,
                               target_area, area_tolerance, pixel_tolerance):    
    """
    Returns True if the tag center is within pixel tolerance of the target offset AND area is within tolerance.
    """
    target_abs_x = frame_width // 2 + target_x_offset_from_frame_center
    target_abs_y = frame_height // 2 + target_y_offset_from_frame_center
    pixel_x_ok = abs(current_tag_center_x_coord - target_abs_x) < pixel_tolerance
    pixel_y_ok = abs(current_tag_center_y_coord - target_abs_y) < pixel_tolerance
    area_ok = abs(current_area - target_area) < area_tolerance
    return pixel_x_ok and pixel_y_ok and area_ok


def position_apriltag_pixel_based(
    tello, current_tag_center_x_coord, current_tag_center_y_coord,
    current_area,
    frame_width, frame_height,
    target_x_offset_from_frame_center,
    target_y_offset_from_frame_center,
    target_area, area_tolerance,
    pError_lr_px, pError_ud_px
):
    """
    Moves the Tello to position the AprilTag's center at a specific pixel offset from the frame center,
    and keeps the tag at a constant area (distance).
    Uses LR, UD, and FB controls.
    """
    target_abs_x = frame_width // 2 + target_x_offset_from_frame_center
    target_abs_y = frame_height // 2 + target_y_offset_from_frame_center

    # LR (left/right) PID
    error_lr_for_pid = target_abs_x - current_tag_center_x_coord
    lr_rc_command = pid_lr[0] * error_lr_for_pid + pid_lr[1] * (error_lr_for_pid - pError_lr_px)
    lr_rc_command = int(np.clip(lr_rc_command, -100, 100))

    # UD (up/down) PID
    error_ud_for_pid = current_tag_center_y_coord - target_abs_y
    ud_rc_command = pid_ud[0] * error_ud_for_pid + pid_ud[1] * (error_ud_for_pid - pError_ud_px)
    ud_rc_command = int(np.clip(ud_rc_command, -100, 100))

    # FB (forward/backward) area-based control
    fb_rc_command = 0
    if current_area > target_area + area_tolerance:
        fb_rc_command = -20  # Move back
    elif current_area < target_area - area_tolerance and current_area != 0:
        fb_rc_command = 20   # Move forward

    # Negate LR and UD as before
    final_lr_command = -lr_rc_command
    final_ud_command = -ud_rc_command
    final_fb_command = fb_rc_command

    tello.send_rc_control(final_lr_command, final_fb_command, final_ud_command, 0)

    return error_lr_for_pid, error_ud_for_pid

# Master function states
STATE_IDLE = 0
STATE_TRACKING = 1
STATE_ALIGNING_YAW = 2
STATE_POSITIONING_PIXEL = 3
STATE_HOLDING = 4

def master_track_and_align_apriltag(
    tello, tag_info, 
    frame_width, frame_height,
    target_area, target_x_offset_px, target_y_offset_px, desired_final_yaw_rad,
    current_state, pid_errors,
    # Optional Tolerances & Parameters
    area_tolerance_tracking = None, 
    offset_tolerance_tracking_px = None, 
    final_yaw_tolerance_rad = None, 
    final_position_tolerance_px = None, 
    align_yaw_power=30,
    align_lr_power=25,
    tracking_fb_speed=20 # Speed for forward/backward during tracking, currently not used by track_apriltag directly
):
    """
    Master function to handle the full sequence of tracking, aligning, and positioning to an AprilTag.

    Args:
        tello: The Tello object.
        tag_info: Dictionary from detect_apriltag or None.
        frame_width: Width of the video frame.
        frame_height: Height of the video frame.
        target_area: Desired AprilTag area for tracking.
        target_x_offset_px: Desired final X pixel offset from frame center.
        target_y_offset_px: Desired final Y pixel offset from frame center.
        desired_final_yaw_rad: Desired final yaw angle in radians.
        current_state: The current operational state (e.g., STATE_TRACKING).
        pid_errors: Dictionary holding previous PID errors:
            {'pError_yaw_track', 'pError_ud_track', 'pError_lr_pixel_pos', 'pError_ud_pixel_pos'}
        area_tolerance_tracking (optional): Tolerance for target_area.
        offset_tolerance_tracking_px (optional): Pixel tolerance for centering during tracking.
        final_yaw_tolerance_rad (optional): Tolerance for final yaw alignment.
        final_position_tolerance_px (optional): Pixel tolerance for final positioning.
        align_yaw_power (optional): Power for yaw movements during alignment.
        align_lr_power (optional): Power for LR movements during alignment.
        tracking_fb_speed (optional): Base speed for FB movements during tracking (Note: track_apriltag internalizes this currently).

    Returns:
        next_state (int): The state for the next iteration.
        pid_errors (dict): Updated PID error values.
        rc_commands_sent (bool): True if RC commands were sent, False otherwise.
    """
    rc_commands_sent = False

    # Set default tolerances if not provided
    if area_tolerance_tracking is None:
        area_tolerance_tracking = max(50, 0.1 * target_area) 
    if offset_tolerance_tracking_px is None:
        offset_tolerance_tracking_px = PIXEL_OFFSET_TOLERANCE_TRACKING 
    if final_yaw_tolerance_rad is None:
        final_yaw_tolerance_rad = 0.05 if desired_final_yaw_rad == 0 else abs(0.1 * desired_final_yaw_rad)
        final_yaw_tolerance_rad = max(0.02, final_yaw_tolerance_rad) 
    if final_position_tolerance_px is None:
        final_position_tolerance_px = POSITIONING_PIXEL_TOLERANCE 

    print("master tolerance = ", offset_tolerance_tracking_px)
    if not tag_info:
        if current_state != STATE_IDLE:
            tello.send_rc_control(0, 0, 0, 0)
            rc_commands_sent = True
        return current_state, pid_errors, rc_commands_sent

    current_tag_center_x = tag_info['center'][0]
    current_tag_center_y = tag_info['center'][1]
    # Ensure 'area' key exists, provide a default or handle if missing
    current_area = tag_info.get('area', 0) # Default to 0 if 'area' is not in tag_info
    # Corrected condition for checking 'angle'
    current_yaw_rad = tag_info['angle'][1] if tag_info.get('angle') is not None and len(tag_info['angle']) > 1 else 0

    offset_x_true_center = current_tag_center_x - frame_width // 2
    offset_y_true_center = current_tag_center_y - frame_height // 2

    next_state = current_state

    if current_state == STATE_TRACKING:
        if not is_tracking_done(current_area, target_area, offset_x_true_center, offset_y_true_center, 
                                area_tolerance_tracking, offset_tolerance_tracking_px):
            print(f"Target: {target_area}\tArea: {current_area}")
            pid_errors['pError_yaw_track'], pid_errors['pError_ud_track'] = track_apriltag(
                tello, current_area, 
                offset_x_true_center, offset_y_true_center, 
                pid_errors['pError_yaw_track'], pid_errors['pError_ud_track'],
                target_area, area_tolerance_tracking
                # Note: track_apriltag uses global AREA_TOLERANCE_TRACKING for fb_speed logic internally
                # and its own hardcoded fb_speed values. The tracking_fb_speed parameter is not directly used by it yet.
            )
            rc_commands_sent = True
        else:
            tello.send_rc_control(0,0,0,0)
            rc_commands_sent = True
            next_state = STATE_ALIGNING_YAW

    elif current_state == STATE_ALIGNING_YAW:
        if not is_aligning_done(current_yaw_rad, desired_final_yaw_rad, final_yaw_tolerance_rad):
            aligning_angle(tello, current_yaw_rad, desired_final_yaw_rad, final_yaw_tolerance_rad, 
                           yaw_power=align_yaw_power, lr_power=align_lr_power)
            rc_commands_sent = True
        else:
            tello.send_rc_control(0,0,0,0)
            rc_commands_sent = True
            next_state = STATE_POSITIONING_PIXEL
            pid_errors['pError_lr_pixel_pos'] = 0 
            pid_errors['pError_ud_pixel_pos'] = 0
            
    elif current_state == STATE_POSITIONING_PIXEL:
        if not is_positioning_pixel_done(current_tag_center_x, current_tag_center_y, 
                                         current_area,
                                         frame_width, frame_height, 
                                         target_x_offset_px, target_y_offset_px, 
                                         target_area, area_tolerance_tracking,
                                         final_position_tolerance_px):
            pid_errors['pError_lr_pixel_pos'], pid_errors['pError_ud_pixel_pos'] = position_apriltag_pixel_based(
                tello, current_tag_center_x, current_tag_center_y,
                current_area,
                frame_width, frame_height,
                target_x_offset_px, target_y_offset_px,
                target_area, area_tolerance_tracking,
                pid_errors['pError_lr_pixel_pos'], pid_errors['pError_ud_pixel_pos']
            )
            rc_commands_sent = True
        else:
            tello.send_rc_control(0,0,0,0)
            rc_commands_sent = True
            next_state = STATE_HOLDING
            
    elif current_state == STATE_HOLDING:
        tello.send_rc_control(0,0,0,0) 
        rc_commands_sent = True
        
    elif current_state == STATE_IDLE:
        pass 

    return next_state, pid_errors, rc_commands_sent

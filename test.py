from main import * # Imports Tello, cv2, Detector, np, and all functions from other modules
# Ensure state constants from track_and_align are available, e.g.
# from track_and_align import STATE_IDLE, STATE_TRACKING, STATE_ALIGNING_YAW, STATE_POSITIONING_PIXEL, STATE_HOLDING

# Target parameters
TARGET_X_OFFSET_PX = -166 
TARGET_Y_OFFSET_PX = -106
TARGET_AREA = 11000 
DESIRED_FINAL_YAW_RAD = 0.80 # Desired final yaw angle in radians (0 means straight at tag)

# Tolerances for specific stages in test.py, to be passed to master function
# These can override defaults in master_track_and_align_apriltag if provided
TEST_FINAL_YAW_TOLERANCE_RAD = 0.15 # Radians, approx 8.6 degrees
TEST_FINAL_POSITION_TOLERANCE_PX = 10 # Pixels

# Global state variables
drone_took_off = False
tag1_state = STATE_IDLE # Use state constants

# PID previous errors dictionary
pid_errors = {
    'pError_yaw_track': 0,
    'pError_ud_track': 0,
    'pError_lr_pixel_pos': 0,
    'pError_ud_pixel_pos': 0
}

if __name__ == "__main__":
    
    tello = init_tello()
    print("Tello initialized and ready. Streaming video...")

    counter_hold_position = 0 # Counter for holding position after all stages

    # State descriptions for display
    state_description = {
        STATE_IDLE: "Idle",
        STATE_TRACKING: "Tracking",
        STATE_ALIGNING_YAW: "Aligning Yaw",
        STATE_POSITIONING_PIXEL: "Positioning Pixel",
        STATE_HOLDING: "Holding Position"
    }

    while True:
        frame_read = tello.get_frame_read()
        if frame_read is None or frame_read.frame is None:
            sleep(0.01) 
            continue
        frame = frame_read.frame
        frame = cv2.resize(frame, (960, 720))
        frame_height, frame_width = frame.shape[:2]

        detected_tags = detect_apriltag(frame, detector) # This now returns a list of dicts or an empty list
        
        # Prepare tag_info for master function (assuming we focus on the first detected tag)
        # The master function expects a single tag_info dictionary or None
        processed_tag_info = None 
        if detected_tags: # If any tags were detected
            # For simplicity, let's use the first detected tag for drawing and control
            # In a multi-tag scenario, you might select a specific tag_id or the largest tag
            processed_tag_info = detected_tags[0] # TODO

            # Call the drawing function with the selected tag's data
            for tag in detected_tags:
                draw_apriltag_info(frame, tag, frame_width, frame_height, TARGET_X_OFFSET_PX, TARGET_Y_OFFSET_PX, TARGET_AREA)

        # Update display text based on current state
        current_state_text = f"State: {tag1_state} ({state_description.get(tag1_state, 'Unknown')})"
        cv2.putText(frame, current_state_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(frame, f"Battery: {tello.get_battery()}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Tello AprilTag Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('t') and not drone_took_off:
            tello.takeoff()
            sleep(1) 
            tello.send_rc_control(0, 0, 0, 0)
            drone_took_off = True
            tag1_state = STATE_TRACKING # Start with tracking state
            # Reset PID errors at takeoff for a fresh start
            for k_err in pid_errors: pid_errors[k_err] = 0
            print(f"Tello took off. Initializing to State {STATE_TRACKING} ({state_description[STATE_TRACKING]}).")
        elif key == ord('l'): 
            print("Landing command received.")
            tello.land() # Land command
            sleep(2) # Give time to land
            drone_took_off = False # Set to false after land confirmed or initiated
            tag1_state = STATE_IDLE # Reset to idle
            

        if drone_took_off:
            # Call the master function
            # It handles cases where processed_tag_info is None
            next_master_state, updated_pid_errors, rc_sent = master_track_and_align_apriltag(
                tello, 
                processed_tag_info, # Pass the tag_info with 'area' or None
                frame_width, frame_height,
                TARGET_AREA, TARGET_X_OFFSET_PX, TARGET_Y_OFFSET_PX, DESIRED_FINAL_YAW_RAD,
                tag1_state, pid_errors,
                # Optional parameters from test.py constants
                final_yaw_tolerance_rad = TEST_FINAL_YAW_TOLERANCE_RAD,
                final_position_tolerance_px = TEST_FINAL_POSITION_TOLERANCE_PX
                # area_tolerance_tracking and offset_tolerance_tracking_px will use master function defaults
            )
            tag1_state = next_master_state
            pid_errors = updated_pid_errors

            # Handle post-master_function logic, like exiting HOLDING state
            if tag1_state == STATE_HOLDING:
                # master_track_and_align_apriltag sends hover in STATE_HOLDING
                # print(f"State {STATE_HOLDING} ({state_description[STATE_HOLDING]}): Holding. Cycle: {counter_hold_position}")
                counter_hold_position += 1
                if counter_hold_position >= 90: # Hold for ~3 seconds (approx 30 FPS * 3s)
                    print(f"State {STATE_HOLDING} ({state_description[STATE_HOLDING]}): Held position. Landing.")
                    # tello.land() # Initiate landing
                    # sleep(2) # Allow time for landing to start
                    drone_took_off = False # This will trigger landing in the main loop exit logic
                    tag1_state = STATE_IDLE # Reset state
                    counter_hold_position = 0 # Reset counter
            else:
                counter_hold_position = 0 # Reset if not in holding state

        # If drone_took_off is false (landed or q pressed), prepare to exit loop
        if not drone_took_off and tello.is_flying: # If flag is false but drone is still airborne (e.g. after hold ends)
             print("Drone_took_off is false, ensuring Tello lands.")
             tello.land()
        
        if key == ord('q'): # Ensure q always allows breaking the loop
             break

    print("Exiting control loop.")
    if tello.is_flying:
        print("Landing Tello...")
        tello.land()
    cv2.destroyAllWindows()
    tello.end()
    print("Tello resources released.")
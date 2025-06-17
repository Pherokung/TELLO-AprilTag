from djitellopy import Tello
import time
from time import sleep
import math
import cv2
from pupil_apriltags import Detector
import numpy as np
from movement_func import *
from april_tag import *
from helper_func import *
from track_and_align import *

detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# Target parameters
TARGET_AREA = 2566  # Target pixel area for the AprilTag
CENTER_THRESHOLD = 20  # How close to center we want to be (in pixels)

# PID controller constants (tune these as needed)
KP_AREA = 0.1  # Forward/backward control
KP_CENTER = 0.2  # Left/right control

drone_took_off = False
tag1 = 0
pError_yaw = 0
pError_ud = 0

def init_tello():
    """
    Initializes and connects to the Tello drone.
    Prints the battery level and returns the drone object.
    Returns:
        Tello: The connected Tello drone instance.
    """
    drone = Tello()
    drone.connect()
    print("Battery:", drone.get_battery())
    #drone.set_video_direction(drone.CAMERA_DOWNWARD)
    drone.streamon()
    
    return drone

if __name__ == "__main__":
    
    tello = init_tello()
    print("Tello initialized and ready. Streaming video...")

    counter_1 = 0

    while True:

        # Read frame from Tello
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (960, 720))
        frame_height, frame_width = frame.shape[:2]

        tag_info = detect_apriltag(frame, detector)
        if tag_info:
            tag_id = tag_info['tag_id']
            center = tag_info['center']
            corners = tag_info['corners']
            pose_t = tag_info['pose_t']
            pose_R = tag_info['pose_R']
            angle = tag_info['angle']

            area = calculate_area(corners)
            area_error = TARGET_AREA - area
            center_x, center_y = center
            offset_x, offset_y = calculate_center_offset(center, frame_width, frame_height)

            # Draw tag corners and center
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {tag_id}", (center[0] - 10, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            #cv2.putText(frame, f"Pos: ({float(pose_t[0]):.2f}, {float(pose_t[1]):.2f}, {float(pose_t[2]):.2f})", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Area: ({area}, {TARGET_AREA})", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Angle & tag: ({angle[1]}, {tag1})", (center[0] - 10, center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            #cv2.putText(frame, f"Tag1: ({tag1})", (center[0] - 10, center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(frame, f"Battery: {tello.get_battery()}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display the frame with AprilTag detection
        cv2.imshow("Tello AprilTag Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('t'):
            tello.takeoff()
            sleep(1)
            drone_took_off = True
            print("Tello took off.")

        if drone_took_off:
            if tag_info:
                # Move towards the tag
                angle_threshold = 0.20
                center_error_x = pose_t[0]  # meters, left/right translation
                center_error_y = pose_t[1]  # meters, up/down translation
                center_threshold = 0.10  # meters (10cm)
                z_target = 3  # meters
                z_error = pose_t[2] - z_target
                # Check alignment
                
                if tag1 != 4:   
                    if not is_tracking_done(area, TARGET_AREA, pError_yaw, pError_ud):
                        print("Tag 1: Not tracked")
                        pError_yaw, pError_ud = track_apriltag(
                            tello, area, 
                            offset_x, offset_y, 
                            pError_yaw, pError_ud,
                            TARGET_AREA
                            )
                        tag1 = 1
                    elif not is_aligning_done(angle, 0.2):
                        print("Tag 1: Not aligned")
                        aligning_angle(tello, angle, 0.2)
                        tag1 = 2
                    else:
                        print(f"Tag 1: In Position for {counter_1}!")
                        counter_1 += 1
                        if counter_1 >= 60:
                            tag1 = 4
                        else:
                            tag1 = 3
                        tello.send_rc_control(0, 0, 0, 0)
                else:
                    move_tello(tello, "forward", 200, 30)
                    compound_move_tello(tello, "up", 80, "forward", 150, 30)
                    curve_tello(tello, 75, -75, 0, 150, 0, 0, 30)
                    curve_tello(tello, 75, 75, 0, 150, 0, 0, 30)
                    break
            else:
                tello.send_rc_control(0, 0, 0, 0)
                # tello.land()

    tello.land()
    cv2.destroyAllWindows()
    tello.end()
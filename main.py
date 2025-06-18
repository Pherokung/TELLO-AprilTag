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
from pathing import pathing

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
stage = 1
state = 0
pid_errors = {
    'pError_yaw_track': 0,
    'pError_ud_track': 0,
    'pError_lr_pixel_pos': 0,
    'pError_ud_pixel_pos': 0
}
call_again = 0


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
    drone.send_rc_control(0, 0, 0, 0)
    
    return drone

if __name__ == "__main__":
    
    tello = init_tello()
    print("Tello initialized and ready. Streaming video...")

    while True:

        # Read frame from Tello
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (960, 720))
        frame_height, frame_width = frame.shape[:2]
        frame_siz = [frame_width, frame_height]

        tag_info = detect_apriltag(frame, detector)
        for tag in tag_info:
            draw_apriltag_info(frame, tag, frame_width, frame_height,)
        
        cv2.putText(frame, f"Battery: {tello.get_battery()}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Stage: {stage}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"State: {state}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display the frame with AprilTag detection
        cv2.imshow("Tello AprilTag Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            drone_took_off = False
            break
        elif key == ord('t'):
            tello.takeoff()
            sleep(1)
            drone_took_off = True
            call_again = time.time()
            print("Tello took off.")

        if drone_took_off:
            if time.time() >= call_again:
                stage, flight_time, state, pid_errors = pathing(tello, stage, tag_info, state, pid_errors, frame_siz)
                call_again = time.time() + flight_time
        
        if stage == 6:
            break

    tello.land()
    cv2.destroyAllWindows()
    tello.end()
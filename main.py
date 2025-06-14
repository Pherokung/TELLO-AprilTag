from djitellopy import Tello
import time
from time import sleep
import math
import cv2
from pupil_apriltags import Detector
import numpy as np

# Helper Functions

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def eul2rot(theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R

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
TARGET_AREA = 50000  # Target pixel area for the AprilTag
AREA_TOLERANCE = 5000  # Acceptable range around target area
CENTER_THRESHOLD = 50  # How close to center we want to be (in pixels)

# PID controller constants (tune these as needed)
KP_AREA = 0.1  # Forward/backward control
KP_CENTER = 0.2  # Left/right control

drone_took_off = False
tag1 = False

pError_yaw = 0

pError_ud = 0

def calculate_area(corners):
    """Calculate area of quadrilateral formed by tag corners"""
    return cv2.contourArea(corners.reshape((-1, 1, 2)))

def calculate_center_offset(center, frame_width, frame_height):
    """Calculate how far tag is from center of frame"""
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2
    return (center[0] - frame_center_x, center[1] - frame_center_y)

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

# Tello Movement Functions

def move_tello(drone, direction, distance, speed = 50):
    """
    Moves the Tello drone in a specified direction for a given distance at a given speed.
    Args:
        drone (Tello): The Tello drone instance.
        direction (str): Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down').
        distance (float): Distance to move in cm.
        speed (int, optional): Speed in cm/s. Default is 50.
    """
    print(f"Tello going {direction}, {distance}cm")

    time = distance / speed

    if direction == "forward":
        drone.send_rc_control(0, speed, 0, 0)
    elif direction == "backward":
        drone.send_rc_control(0, -speed, 0, 0)
    elif direction == "left":
        drone.send_rc_control(-speed, 0, 0, 0)
    elif direction == "right":
        drone.send_rc_control(speed, 0, 0, 0)
    elif direction == "up":
        drone.send_rc_control(0, 0, speed, 0)
    elif direction == "down":
        drone.send_rc_control(0, 0, -speed, 0)

    sleep(time)

def compound_move_tello(drone, direction1, distance1, direction2, distance2, speed=50):
    print(f"Moving in {direction1} {distance1}cm and {direction2} {distance2}cm")
    
    total_distance = math.sqrt(distance1**2 + distance2**2)
    total_time = total_distance / speed

    speed1 = int(distance1 / total_time)
    speed2 = int(distance2 / total_time)

    x, y, z = 0, 0, 0
    
    if direction1 == "forward": 
        y = speed1
    elif direction2 == "forward":
        y = speed2

    if direction1 == "backward": 
        y = -speed1
    elif direction2 == "backward":
        y = -speed2

    if direction1 == "right": 
        x = speed1
    elif direction2 == "right":
        x = speed2

    if direction1 == "left": 
        x = -speed1
    elif direction2 == "left":
        x = -speed2

    if direction1 == "up": 
        z = speed1
    elif direction2 == "up":
        z = speed2

    if direction1 == "down": 
        z = -speed1
    elif direction2 == "down":
        z = -speed2

    drone.send_rc_control(x, y, z, 0)

    return total_time


def move_tello_bearing(drone, bearing, distance, speed = 50):
    """
    Moves the Tello drone in a specified bearing for a given distance at a given speed.
    Args:
        drone (Tello): The Tello drone instance.
        bearing (float): Bearing in degrees (0-360).
        distance (float): Distance to move in cm.
        speed (int, optional): Speed in cm/s. Default is 50.
    """
    if not 0 <= bearing < 360:
        raise ValueError("Bearing must be between 0 and 360 degrees.")

    radians = math.radians(bearing)
    x = int(distance * math.cos(radians))
    y = int(distance * math.sin(radians))

    drone.send_rc_control(x, y, 0, 0)
    sleep(distance / speed)

def turn_tello(drone, angle, speed = 30):
    """
    Rotates the Tello drone by a specified angle at a given speed.
    Args:
        drone (Tello): The Tello drone instance.
        angle (float): Angle to turn in degrees. Positive for clockwise, negative for counterclockwise.
        speed (int, optional): Rotational speed in degrees/s. Default is 30.
    """
    time = abs(angle) / speed

    if angle > 0:
        drone.send_rc_control(0, 0, 0, speed)
    else:
        drone.send_rc_control(0, 0, 0, -speed)

    return time

def curve_tello(drone, x1, y1, z1, x2, y2, z2, speed = 50):
    """
    Fly to (x2, y2, z2) in a curve via (x1, y1, z1) at the given speed.
    All coordinates are relative to the current position (cm).
    Raises Exception if parameters are out of range or invalid.
    """
    # Parameter validation
    for v in [x1, y1, z1, x2, y2, z2]:
        if not -500 <= v <= 500:
            raise ValueError("All coordinates must be between -500 and 500.")
    if not 10 <= speed <= 60:
        raise ValueError("Speed must be between 10 and 60 cm/s.")

    # x1/y1/z1 and x2/y2/z2 can't both be between -20 and 20 at the same time (but can both be 0)
    def in_range(a, b):
        return (-20 <= a <= 20) and (-20 <= b <= 20)
    if in_range(x1, x2) and in_range(y1, y2) and in_range(z1, z2):
        if not (x1 == x2 == y1 == y2 == z1 == z2 == 0):
            raise ValueError("x1/x2, y1/y2, z1/z2 can't both be between -20 and 20 at the same time, unless all are 0.")

    # Check arc radius (must be 0.5-10 meters)
    p0 = (0, 0, 0)
    p1 = (x1, y1, z1)
    p2 = (x2, y2, z2)
    # Calculate circle radius from 3 points
    def circle_radius(a, b, c):
        A = math.dist(a, b)
        B = math.dist(b, c)
        C = math.dist(c, a)
        s = (A + B + C) / 2
        area = math.sqrt(max(s * (s - A) * (s - B) * (s - C), 0))
        if area == 0:
            return float('inf')
        return (A * B * C) / (4 * area)
    radius = circle_radius(p0, p1, p2) / 100  # convert cm to m
    if not 0.5 <= radius <= 10:
        raise Exception(f"Arc radius {radius:.2f}m not in range 0.5-10m.")

    # Send curve command
    if hasattr(drone, 'curve_xyz_speed'):
        drone.curve_xyz_speed(x1, y1, z1, x2, y2, z2, speed)
    else:
        # fallback: send command directly
        drone.send_command_with_return(f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed}")

    # Estimate time to complete (distance/speed)
    dist = math.dist(p0, p1) + math.dist(p1, p2)
    return dist / speed

# AprilTag Functions

def detect_apriltag(frame, detector):
    """
    Detect AprilTag in the given frame using the specified detector.
    Returns a list of detected tags with their pose and corners.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=[921.170702, 919.018377, 459.904354, 351.238301],
        tag_size=0.165
    )
    if results:
        r = results[0]
        tag_id = r.tag_id
        center = (int(r.center[0]), int(r.center[1]))
        corners = r.corners.astype(int)
        pose_t = r.pose_t.flatten()
        pose_R = r.pose_R
        angle = rot2eul(pose_R)
        return {
                    'tag_id': tag_id,
                    'center': center,
                    'corners': corners,
                    'pose_t': pose_t,
                    'pose_R': pose_R,
                    'angle': angle
                }
    return None

def track_apriltag(tello, area, ox, oy, pError_yaw, pError_ud, target_area):
    pid_yaw = [0.4, 0.4, 0]
    pid_ud = [0.2, 0.2, 0]

    error_yaw = ox
    speed = pid_yaw[0] * error_yaw + pid_yaw[1] * (error_yaw - pError_yaw)
    speed = int(np.clip(speed, -100, 100))

    if area > target_area - AREA_TOLERANCE and area < target_area + AREA_TOLERANCE:
        fb = 0
    elif area > target_area + AREA_TOLERANCE:
        fb = -20
    elif area < target_area - AREA_TOLERANCE and area != 0:
        fb = 20
    
    error_ud = -oy
    ud = pid_ud[0] * error_ud + pid_ud[1] * (error_ud - pError_ud)
    ud = int(np.clip(ud, -100, 100))

    tello.send_rc_control(0, fb, ud, speed)
    print("----------------")
    print("Up-Down:", ud)
    print("Yaw", speed)

    return error_yaw, error_ud
    
if __name__ == "__main__":
    
    tello = init_tello()
    print("Tello initialized and ready. Streaming video...")

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
            cv2.putText(frame, f"Offset: ({offset_x}, {offset_y})", (center[0] - 10, center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

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

        if drone_took_off and not tag1:
            if tag_info:
                # Move towards the tag
                angle_threshold = 0.20
                center_error_x = pose_t[0]  # meters, left/right translation
                center_error_y = pose_t[1]  # meters, up/down translation
                center_threshold = 0.10  # meters (10cm)
                z_target = 3  # meters
                z_error = pose_t[2] - z_target
                # Check alignment
                
                pError_yaw, pError_ud = track_apriltag(
                    tello, area, 
                    offset_x, offset_y, 
                    pError_yaw, pError_ud,
                    TARGET_AREA
                    )
            else:
                tello.send_rc_control(0, 0, 0, 0)
        else:
            tello.send_rc_control(0, 0, 0, 0)

    cv2.destroyAllWindows()
    tello.end()
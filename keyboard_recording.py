import cv2
import threading
from djitellopy import Tello
from pupil_apriltags import Detector
import numpy as np
import json
import time
import pygame
# Updated imports:
from april_tag import calculate_area, detect_apriltag, calculate_center_offset
# helper_func.rot2eul is used within detect_apriltag

# AprilTag detector setup

detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# Tello setup

def init_tello():
    drone = Tello()
    drone.connect()
    print("Battery:", drone.get_battery())
    drone.streamon()
    return drone

# Updated Data structure comment
# Data structure for in-memory storage:
# {
#   user_tag_num (int): [ # Keyed by 1-9 from keyboard
#     { # List of individual detections
#       'tag_id': int,        # Actual AprilTag ID
#       'center': (int, int),
#       'corners': np.array,
#       'pose_t': np.array,   # Translation vector
#       'pose_R': np.array,   # Rotation matrix
#       'angle': (float, float, float), # Roll, Pitch, Yaw
#       'area': float,
#       'offset_x': int,
#       'offset_y': int
#     }, ...
#   ], ...
# }
# JSON_FILE_PATH and load_records remain the same
JSON_FILE_PATH = "tello_tag_records.json"

def load_records():
    try:
        with open(JSON_FILE_PATH, 'r') as f:
            data = json.load(f)
            # Data is loaded as per JSON structure, no complex conversion needed here
            # as record_dict will be populated by new recordings if this is empty.
            # If you need to load and re-process old JSON, this might need adjustment
            # based on whether old JSON matches the new richer structure.
            # For now, assume we are primarily concerned with saving new format.
            # And if an old format is loaded, it might not be fully compatible
            # with parts of the code expecting the new richer dict structure.
            # A robust solution might involve versioning or checking keys.
            # For this modification, we focus on saving the new structure.
            # The loaded 'data' will be used as the initial 'record_dict'.
            # If it's an old format, new recordings will use the new format.
            # Let's convert string keys from JSON back to int for record_dict
            return {int(k): v for k, v in data.items()} if data else {}
    except FileNotFoundError:
        print(f"Info: {JSON_FILE_PATH} not found. Starting with an empty record set.")
        return {}
    except json.JSONDecodeError:
        print(f"Error: Could not decode {JSON_FILE_PATH}. Starting with an empty record set.")
        return {}

record_dict = load_records() # Restore loading of records
record_lock = threading.Lock()

# Helper to get average of list of np arrays (remains useful)

def average_np(arrs):
    arrs = np.array(arrs)
    return np.mean(arrs, axis=0)

# Video/AprilTag/Recording thread
def video_apriltag_thread(drone, stop_event, latest_tag_data_shared): # Renamed for clarity
    while not stop_event.is_set():
        frame = drone.get_frame_read().frame
        frame_resized = cv2.resize(frame, (960, 720)) # Use a different variable for resized frame
        frame_height, frame_width = frame_resized.shape[:2]

        # Use detect_apriltag from april_tag.py
        tag_info_dict = detect_apriltag(frame_resized, detector) # detector is global

        current_detection_details = None
        if tag_info_dict:
            # Calculate area
            area = calculate_area(tag_info_dict['corners'])
            # Calculate center offset
            offset_x, offset_y = calculate_center_offset(tag_info_dict['center'], frame_width, frame_height)

            # Prepare the full data dictionary for this detection
            current_detection_details = {
                'tag_id': tag_info_dict['tag_id'],
                'center': tag_info_dict['center'],
                'corners': tag_info_dict['corners'], # Keep as numpy array for potential use
                'pose_t': tag_info_dict['pose_t'],
                'pose_R': tag_info_dict['pose_R'],
                'angle': tag_info_dict['angle'],
                'area': area,
                'offset_x': offset_x,
                'offset_y': offset_y
            }

            # Drawing on frame (similar to before, but using data from current_detection_details)
            r_center = current_detection_details['center']
            r_corners = current_detection_details['corners'].astype(int) # Ensure corners are int for drawing
            r_tag_id = current_detection_details['tag_id']
            r_pose_t = current_detection_details['pose_t']

            for i in range(4):
                cv2.line(frame_resized, tuple(r_corners[i]), tuple(r_corners[(i+1) % 4]), (0, 255, 0), 2)
            cv2.circle(frame_resized, r_center, 5, (0, 0, 255), -1)
            cv2.putText(frame_resized, f"ID: {r_tag_id}", (r_center[0] - 10, r_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame_resized, f"Pos: ({float(r_pose_t[0]):.2f}, {float(r_pose_t[1]):.2f}, {float(r_pose_t[2]):.2f})", (r_center[0] - 10, r_center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame_resized, f"Area: {area:.2f}", (r_center[0] - 10, r_center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame_resized, f"Offset: ({offset_x}, {offset_y})", (r_center[0] - 10, r_center[1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        latest_tag_data_shared[0] = current_detection_details # Store the dict or None
        # print(latest_tag_data_shared[0]) # Optional: for debugging
        cv2.imshow("Tello Key Record", frame_resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break
    cv2.destroyAllWindows()

def pygame_control_thread(drone, stop_event, record_dict_ref, record_lock_ref, get_latest_tag_data_func): # Pass refs and func
    pygame.init()
    # ... (Pygame setup remains the same) ...
    WIDTH, HEIGHT = 400, 200
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Tello Controls & Recording")
    font = pygame.font.SysFont(None, 28)
    speed = 50
    flying = False
    info = ""
    clock = pygame.time.Clock()

    while not stop_event.is_set():
        # ... (Pygame event handling and display updates) ...
        WIN.fill((30, 30, 30))
        text = font.render("Controls: e=takeoff, l=land, wasd/ijkl=move, 1-9=record, q=quit", True, (255,255,255))
        WIN.blit(text, (10, 10))
        text2 = font.render(info, True, (0,255,0))
        WIN.blit(text2, (10, 50))
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()
                return
            if event.type == pygame.KEYDOWN:
                key = event.key
                if key == pygame.K_q:
                    stop_event.set()
                    return
                elif key == pygame.K_e and not flying:
                    drone.takeoff()
                    flying = True
                    info = "Takeoff"
                elif key == pygame.K_l and flying:
                    drone.land()
                    flying = False
                    info = "Land"
                elif key == pygame.K_w: # Assuming 'w' is for up, not forward based on common game controls
                    drone.send_rc_control(0, 0, speed, 0) # lr, fb, ud, yaw
                    info = "Up"
                elif key == pygame.K_s:
                    drone.send_rc_control(0, 0, -speed, 0)
                    info = "Down"
                elif key == pygame.K_a: # Yaw left
                    drone.send_rc_control(0, 0, 0, -speed)
                    info = "Yaw Left"
                elif key == pygame.K_d: # Yaw right
                    drone.send_rc_control(0, 0, 0, speed)
                    info = "Yaw Right"
                elif key == pygame.K_UP: # Forward
                    drone.send_rc_control(0, speed, 0, 0)
                    info = "Forward"
                elif key == pygame.K_DOWN: # Backward
                    drone.send_rc_control(0, -speed, 0, 0)
                    info = "Backward"
                elif key == pygame.K_LEFT: # Left
                    drone.send_rc_control(-speed, 0, 0, 0)
                    info = "Left"
                elif key == pygame.K_RIGHT: # Right
                    drone.send_rc_control(speed, 0, 0, 0)
                    info = "Right"

                elif pygame.K_1 <= key <= pygame.K_9:
                    user_tag_num = key - pygame.K_0 # This is the 1-9 key
                    
                    latest_tag_data = get_latest_tag_data_func() # This gets the dictionary
                    
                    if latest_tag_data: # Check if a tag was detected
                        # Make copies of numpy arrays before storing
                        data_to_store = latest_tag_data.copy() # Shallow copy is fine for top-level dict
                        data_to_store['pose_t'] = latest_tag_data['pose_t'].copy()
                        data_to_store['pose_R'] = latest_tag_data['pose_R'].copy()
                        if isinstance(latest_tag_data['corners'], np.ndarray):
                             data_to_store['corners'] = latest_tag_data['corners'].copy()
                        # 'center' and 'angle' are tuples, 'tag_id', 'area', 'offset_x', 'offset_y' are primitives - no deep copy needed

                        with record_lock_ref:
                            # If the current entry for user_tag_num is not a list (e.g., it's a loaded summary dict),
                            # or if the key doesn't exist, initialize it as a new list for raw data.
                            if user_tag_num not in record_dict_ref or not isinstance(record_dict_ref[user_tag_num], list):
                                record_dict_ref[user_tag_num] = []
                            record_dict_ref[user_tag_num].append(data_to_store)
                            print(f"Recorded data for user tag num {user_tag_num}: "
                                  f"Actual Tag ID {data_to_store['tag_id']}, Area {data_to_store['area']:.2f}")
                            save_records_to_json(record_dict_ref) # Save after each recording
                            info = f"Recorded for user tag {user_tag_num}"
                    else:
                        info = f"No AprilTag in view to record for {user_tag_num}"
                        print(f"Attempted to record for user tag {user_tag_num}, but no AprilTag detected.")
        
        # Stop movement if no key is pressed (remains the same)
        # ...existing code...
        keys = pygame.key.get_pressed()
        if not any(keys): # This check might be too simplistic if some keys are for recording/takeoff/land
            # More precise check: if no movement keys are pressed
            movement_keys = [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, 
                             pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT]
            if not any(keys[k] for k in movement_keys):
                 drone.send_rc_control(0, 0, 0, 0)
        clock.tick(30)
    pygame.quit()

def save_records_to_json(records_to_save): # Renamed parameter
    out_json = {}
    for user_tag_num_key, data_for_tag in records_to_save.items(): # user_tag_num_key is int from record_dict
        user_tag_num_str = str(user_tag_num_key) # JSON keys must be strings

        if isinstance(data_for_tag, list): # This is raw data recorded (or re-recorded) in this session
            recorded_data_list = data_for_tag
            if not recorded_data_list: # Should not happen if we append, but good check
                print(f"Info: No raw data to save for user tag {user_tag_num_str}.")
                continue

            num_records = len(recorded_data_list)
            # All items in recorded_data_list are expected to be dicts with 'tag_id' etc.
            actual_tag_id = recorded_data_list[0]['tag_id']

            avg_translation = np.mean([rec['pose_t'] for rec in recorded_data_list], axis=0).tolist()
            avg_rotation_matrix = np.mean([rec['pose_R'] for rec in recorded_data_list], axis=0).tolist()
            avg_angle = np.mean([rec['angle'] for rec in recorded_data_list], axis=0).tolist()
            avg_area = float(np.mean([rec['area'] for rec in recorded_data_list]))
            
            avg_center_x = float(np.mean([rec['center'][0] for rec in recorded_data_list]))
            avg_center_y = float(np.mean([rec['center'][1] for rec in recorded_data_list]))
            avg_center = [avg_center_x, avg_center_y]

            avg_offset_x = float(np.mean([rec['offset_x'] for rec in recorded_data_list]))
            avg_offset_y = float(np.mean([rec['offset_y'] for rec in recorded_data_list]))
            avg_offset = [avg_offset_x, avg_offset_y]

            out_json[user_tag_num_str] = {
                "num_records": num_records,
                "actual_tag_id": actual_tag_id,
                "avg_translation": avg_translation,
                "avg_rotation_matrix": avg_rotation_matrix,
                "avg_angle": avg_angle,
                "avg_area": avg_area,
                "avg_center": avg_center,
                "avg_offset": avg_offset
            }
        elif isinstance(data_for_tag, dict): # This is a summary loaded and not touched in this session
            # Pass through the existing summary dictionary.
            # Ensure all its values are JSON serializable if they weren't already.
            # (load_records should handle this if JSON was valid)
            out_json[user_tag_num_str] = data_for_tag
            print(f"Info: Passing through existing summary for user tag {user_tag_num_str}.")
        else:
            print(f"Warning: Unexpected data type for user tag {user_tag_num_str} in save_records_to_json. Skipping.")
            continue
            
    try:
        with open(JSON_FILE_PATH, 'w') as f:
            json.dump(out_json, f, indent=2) # Use indent for readability
        print(f"Records saved to {JSON_FILE_PATH}")
    except Exception as e:
        print(f"Error saving records to JSON: {e}")


# Main execution
if __name__ == '__main__':
    drone = init_tello()
    stop_event = threading.Event()
    
    # Shared data structure for the latest detected tag's full details
    # This will hold a dictionary or None
    latest_tag_data_shared = [None] 

    video_thread = threading.Thread(target=video_apriltag_thread, args=(drone, stop_event, latest_tag_data_shared))
    
    # Lambda to pass to pygame thread to get the latest data
    get_latest_tag_data_func = lambda: latest_tag_data_shared[0]

    control_thread = threading.Thread(target=pygame_control_thread, args=(drone, stop_event, record_dict, record_lock, get_latest_tag_data_func))

    video_thread.start()
    control_thread.start()

    video_thread.join()
    control_thread.join()

    if drone.is_flying:
        drone.land()
    drone.streamoff()
    drone.end()
    print("Program ended.")

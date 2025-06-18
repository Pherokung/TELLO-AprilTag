import cv2
import threading
from djitellopy import Tello
from pupil_apriltags import Detector
import numpy as np
import json
import time
import pygame
# Updated imports:
from april_tag import detect_apriltag, calculate_center_offset, calculate_area # Explicitly import
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
# In-memory storage: record_dict
# {
#   user_tag_num (int, e.g., 1-9 from keyboard): {
#     actual_tag_id (int, e.g., 5): [ # List of individual detections for this specific AprilTag ID
#       {
#         'tag_id': int,
#         'center': (int, int),
#         'corners': np.array, # Shape (4, 2)
#         'pose_t': np.array,  # Translation vector (3,)
#         'pose_R': np.array,  # Rotation matrix (3, 3)
#         'angle': (float, float, float), # Roll, Pitch, Yaw
#         'area': float,
#         'offset_x': int,     # Offset from frame center X
#         'offset_y': int      # Offset from frame center Y
#       },
#       ... # more detections for this actual_tag_id under this user_tag_num
#     ],
#     actual_tag_id_2 (int, e.g., 8): [ ... ],
#     ... # more actual AprilTag IDs detected when user_tag_num was pressed
#   },
#   user_tag_num_2 (int): { ... },
#   ...
# }

# JSON file structure (tello_tag_records.json):
# {
#   "user_tag_num_str" (e.g., "1"): {
#     "actual_tag_id_str" (e.g., "5"): {
#       "num_records": int,
#       "avg_translation": [float, float, float],
#       "avg_rotation_matrix": [[float,...],[...],[...]],
#       "avg_angle": [float, float, float],
#       "avg_area": float,
#       "avg_center": [float, float], # Averaged (x, y)
#       "avg_offset": [float, float]  # Averaged (offset_x, offset_y)
#     },
#     "actual_tag_id_str_2" (e.g., "8"): { ... },
#     ...
#   },
#   "user_tag_num_str_2": { ... },
#   ...
# }
JSON_FILE_PATH = "tello_tag_records.json"

def load_records():
    try:
        with open(JSON_FILE_PATH, 'r') as f:
            data = json.load(f)
            # Convert top-level keys (user_tag_num) to int.
            # The inner structure (actual_tag_id summaries) remains as loaded.
            # This loaded data is primarily for reference or if no new recordings are made
            # for a specific user_tag_num. If new recordings happen, they'll use the
            # detailed list-of-detections format in memory.
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
def video_apriltag_thread(drone, stop_event, latest_tag_data_shared):
    while not stop_event.is_set():
        frame = drone.get_frame_read().frame
        if frame is None:
            time.sleep(0.01) # Wait if frame is not available
            continue
            
        frame_resized = cv2.resize(frame, (960, 720))
        frame_height, frame_width = frame_resized.shape[:2]

        # detect_apriltag returns a list of dictionaries, each with 'area' already calculated
        detected_tags_current_frame = detect_apriltag(frame_resized, detector)
        
        processed_tags_for_sharing = []
        if detected_tags_current_frame:
            for tag_data in detected_tags_current_frame:
                # Calculate center offset for this tag
                offset_x, offset_y = calculate_center_offset(tag_data['center'], frame_width, frame_height)
                
                # Add offsets to the tag's dictionary (create a copy to avoid modifying original from detect_apriltag if it's reused)
                current_tag_processed = tag_data.copy()
                current_tag_processed['offset_x'] = offset_x
                current_tag_processed['offset_y'] = offset_y
                processed_tags_for_sharing.append(current_tag_processed)

                # Drawing on frame (using data from current_tag_processed)
                r_center = current_tag_processed['center']
                # Ensure corners are int for drawing, it should be float from detection for area calc
                r_corners = current_tag_processed['corners'].astype(int) 
                r_tag_id = current_tag_processed['tag_id']
                r_pose_t = current_tag_processed['pose_t']
                r_area = current_tag_processed['area'] # Area is already in tag_data

                for i in range(4):
                    cv2.line(frame_resized, tuple(r_corners[i]), tuple(r_corners[(i + 1) % 4]), (0, 255, 0), 2)
                cv2.circle(frame_resized, r_center, 5, (0, 0, 255), -1)
                cv2.putText(frame_resized, f"ID: {r_tag_id}", (r_center[0] - 10, r_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame_resized, f"Pos: ({float(r_pose_t[0]):.2f}, {float(r_pose_t[1]):.2f}, {float(r_pose_t[2]):.2f})", (r_center[0] - 10, r_center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
                cv2.putText(frame_resized, f"Area: {r_area:.0f}", (r_center[0] - 10, r_center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
                cv2.putText(frame_resized, f"Offset: ({offset_x}, {offset_y})", (r_center[0] - 10, r_center[1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
        else:
            # No tags detected this frame
            pass # No need to print "No Tags" every frame, can be noisy

        latest_tag_data_shared[0] = processed_tags_for_sharing if processed_tags_for_sharing else None
        
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

                elif (pygame.K_1 <= key <= pygame.K_9) or key == pygame.K_0:
                    if key == pygame.K_0:
                        user_tag_num = 10
                    else:
                        user_tag_num = key - pygame.K_0 # 1-9 keys
                    
                    all_detected_tags_at_keypress = get_latest_tag_data_func() # This gets the list of tag dicts or None
                    
                    if all_detected_tags_at_keypress: # Check if any tags were detected
                        num_tags_recorded_this_press = 0
                        with record_lock_ref:
                            # Ensure the entry for user_tag_num is a dictionary (to hold actual_tag_ids)
                            if user_tag_num not in record_dict_ref or not isinstance(record_dict_ref[user_tag_num], dict):
                                record_dict_ref[user_tag_num] = {}
                            
                            user_records_for_actual_tags = record_dict_ref[user_tag_num]

                            for tag_data_to_store in all_detected_tags_at_keypress:
                                actual_tag_id = tag_data_to_store['tag_id']
                                
                                # Ensure the entry for this actual_tag_id is a list (to hold individual records)
                                if actual_tag_id not in user_records_for_actual_tags:
                                    user_records_for_actual_tags[actual_tag_id] = []
                                
                                # Make copies of numpy arrays before storing
                                data_copy = tag_data_to_store.copy()
                                data_copy['pose_t'] = tag_data_to_store['pose_t'].copy()
                                data_copy['pose_R'] = tag_data_to_store['pose_R'].copy()
                                if isinstance(tag_data_to_store['corners'], np.ndarray):
                                     data_copy['corners'] = tag_data_to_store['corners'].copy()
                                # Other fields like 'center', 'angle', 'area', 'offset_x', 'offset_y' are primitives or tuples, shallow copy is fine.

                                user_records_for_actual_tags[actual_tag_id].append(data_copy)
                                num_tags_recorded_this_press += 1
                                print(f"  Recorded for user key {user_tag_num}: Actual Tag ID {actual_tag_id}, Area {data_copy['area']:.0f}")

                        if num_tags_recorded_this_press > 0:
                            save_records_to_json(record_dict_ref) # Save after each successful recording event
                            info = f"Rec for {user_tag_num}: {num_tags_recorded_this_press} tag(s)"
                        else: # Should not happen if all_detected_tags_at_keypress was not empty
                            info = f"No new tag data for {user_tag_num}"
                    else:
                        info = f"No AprilTags in view to record for {user_tag_num}"
                        print(f"Attempted to record for user key {user_tag_num}, but no AprilTags detected.")
        
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

def save_records_to_json(records_input): # Renamed parameter for clarity
    out_json = {}
    # records_input is the main record_dict
    for user_tag_num_key, actual_tags_dict in records_input.items():
        user_tag_num_str = str(user_tag_num_key) # JSON keys must be strings
        out_json[user_tag_num_str] = {}

        if not isinstance(actual_tags_dict, dict):
            # This might be an old summary format loaded for a user_tag_num that wasn't re-recorded.
            # Pass it through if it's a dict, otherwise skip.
            if isinstance(actual_tags_dict, dict): # Check if it's a dict (summary)
                 out_json[user_tag_num_str] = actual_tags_dict
                 print(f"Info: Passing through existing summary for user key {user_tag_num_str} (not re-recorded this session).")
            else:
                print(f"Warning: Skipping user key {user_tag_num_str}, data is not in expected dictionary format: {type(actual_tags_dict)}")
            continue

        for actual_tag_id_key, recorded_data_list in actual_tags_dict.items():
            actual_tag_id_str = str(actual_tag_id_key)

            if not isinstance(recorded_data_list, list) or not recorded_data_list:
                # This could be an entry from a loaded summary that wasn't a list,
                # or an empty list if somehow a tag ID was created but no data recorded.
                # If it's a dict, it's part of a loaded summary for this specific actual_tag_id.
                if isinstance(recorded_data_list, dict): # Part of a loaded summary for this actual_tag_id
                    out_json[user_tag_num_str][actual_tag_id_str] = recorded_data_list
                    # print(f"Info: Passing through existing summary for user key {user_tag_num_str}, actual tag ID {actual_tag_id_str}.")
                else:
                    print(f"Info: No raw data to process for user key {user_tag_num_str}, actual tag ID {actual_tag_id_str}. Skipping.")
                continue

            num_records = len(recorded_data_list)
            
            # Ensure all items in recorded_data_list are dicts with expected keys
            # This is important because loaded data might be mixed in if not careful with initialization.
            # However, the recording logic should ensure new recordings are lists of dicts.
            valid_records = [rec for rec in recorded_data_list if isinstance(rec, dict) and 'pose_t' in rec]
            if not valid_records:
                print(f"Warning: No valid records found for user key {user_tag_num_str}, actual tag ID {actual_tag_id_str} for averaging. Skipping.")
                continue
            
            num_records = len(valid_records) # Update num_records based on valid ones

            # All items in valid_records are expected to be dicts with 'tag_id' etc.
            # actual_tag_id is already known from actual_tag_id_key

            avg_translation = np.mean([rec['pose_t'] for rec in valid_records], axis=0).tolist()
            avg_rotation_matrix = np.mean([rec['pose_R'] for rec in valid_records], axis=0).tolist()
            avg_angle = np.mean([rec['angle'] for rec in valid_records], axis=0).tolist()
            avg_area = float(np.mean([rec['area'] for rec in valid_records]))
            
            avg_center_x = float(np.mean([rec['center'][0] for rec in valid_records]))
            avg_center_y = float(np.mean([rec['center'][1] for rec in valid_records]))
            avg_center = [avg_center_x, avg_center_y]

            avg_offset_x = float(np.mean([rec['offset_x'] for rec in valid_records]))
            avg_offset_y = float(np.mean([rec['offset_y'] for rec in valid_records]))
            avg_offset = [avg_offset_x, avg_offset_y]

            out_json[user_tag_num_str][actual_tag_id_str] = {
                "num_records": num_records,
                # "actual_tag_id": actual_tag_id_key, # Redundant, it's the key
                "avg_translation": avg_translation,
                "avg_rotation_matrix": avg_rotation_matrix,
                "avg_angle": avg_angle,
                "avg_area": avg_area,
                "avg_center": avg_center,
                "avg_offset": avg_offset
            }
            
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
    
    # Shared data structure for the latest detected tags' full details
    # This will hold a LIST of tag dictionaries, or None if no tags are detected.
    latest_tag_data_shared = [None] 

    video_thread = threading.Thread(target=video_apriltag_thread, args=(drone, stop_event, latest_tag_data_shared))
    
    # Lambda to pass to pygame thread to get the latest data (list of tags or None)
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

<<<<<<< HEAD
import cv2
import threading
from djitellopy import Tello
from pupil_apriltags import Detector
import numpy as np
import json
import time
import pygame
from april_tag import calculate_area

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

# Data structure: {tag_id: [list of (translation, rotation matrix)]}
record_dict = {}
record_lock = threading.Lock()

# Helper to get average of list of np arrays

def average_np(arrs):
    arrs = np.array(arrs)
    return np.mean(arrs, axis=0)

# Video/AprilTag/Recording thread

def video_apriltag_thread(drone, stop_event, latest_tag_pose):
    while not stop_event.is_set():
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (960, 720))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray, estimate_tag_pose=True, camera_params=[921.170702,919.018377,459.904354,351.238301], tag_size=0.165)
        tag_id = None
        pose = None
        rot = None
        if results:
            r = results[0]
            tag_id = r.tag_id
            center = (int(r.center[0]), int(r.center[1]))
            corners = r.corners.astype(int)
            area = calculate_area(corners)
            pose_t = r.pose_t.flatten()
            pose_R = r.pose_R
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {tag_id}", (center[0] - 10, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Pos: ({float(pose_t[0]):.2f}, {float(pose_t[1]):.2f}, {float(pose_t[2]):.2f})", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            pose = pose_t
            rot = pose_R
        else:
            pose = None
            rot = None
        latest_tag_pose[0] = (pose, rot, area)
        cv2.imshow("Tello Key Record", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break
    cv2.destroyAllWindows()

def pygame_control_thread(drone, stop_event, record_dict, record_lock, get_latest_tag_pose):
    pygame.init()
    WIDTH, HEIGHT = 400, 200
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Tello Controls & Recording")
    font = pygame.font.SysFont(None, 28)
    speed = 50
    flying = False
    info = ""
    clock = pygame.time.Clock()
    while not stop_event.is_set():
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
                elif key == pygame.K_w:
                    drone.send_rc_control(0, 0, speed, 0)
                    info = "Up"
                elif key == pygame.K_s:
                    drone.send_rc_control(0, 0, -speed, 0)
                    info = "Down"
                elif key == pygame.K_a:
                    drone.send_rc_control(0, 0, 0, -speed)
                    info = "Yaw Left"
                elif key == pygame.K_d:
                    drone.send_rc_control(0, 0, 0, speed)
                    info = "Yaw Right"
                elif key == pygame.K_UP:
                    drone.send_rc_control(0, speed, 0, 0)
                    info = "Forward"
                elif key == pygame.K_DOWN:
                    drone.send_rc_control(0, -speed, 0, 0)
                    info = "Backward"
                elif key == pygame.K_LEFT:
                    drone.send_rc_control(-speed, 0, 0, 0)
                    info = "Left"
                elif key == pygame.K_RIGHT:
                    drone.send_rc_control(speed, 0, 0, 0)
                    info = "Right"
                elif pygame.K_1 <= key <= pygame.K_9:
                    tag_num = key - pygame.K_0
                    latest_pose, latest_rot, latest_area = get_latest_tag_pose()
                    if latest_pose is not None and latest_rot is not None:
                        with record_lock:
                            if tag_num not in record_dict:
                                record_dict[tag_num] = []
                            record_dict[tag_num].append((latest_pose.copy(), latest_rot.copy(), latest_area.copy()))
                            print(f"Recorded tag {tag_num}: pos {latest_pose},\nrot:\n{latest_rot}\narea: {latest_area}")
                            save_records_to_json(record_dict)
                            info = f"Recorded tag {tag_num}"
        # Stop movement if no key is pressed
        keys = pygame.key.get_pressed()
        if not any(keys):
            drone.send_rc_control(0, 0, 0, 0)
        clock.tick(30)
    pygame.quit()

def save_records_to_json(record_dict):
    out = {}
    for tag_id, records in record_dict.items():
        if not records:
            continue
        poses = np.array([r[0] for r in records])
        rots = np.array([r[1] for r in records])
        areas = np.array([r[2] for r in records])
        avg_pose = np.mean(poses, axis=0).tolist()
        avg_rot = np.mean(rots, axis=0).tolist()
        avg_area = np.mean(areas)
        out[tag_id] = {
            "num_records": len(records),
            "avg_translation": avg_pose,
            "avg_rotation_matrix": avg_rot,
            "avg_area": avg_area
        }
    with open("tello_tag_records.json", "w") as f:
        json.dump(out, f, indent=2)

if __name__ == "__main__":
    drone = init_tello()
    stop_event = threading.Event()
    latest_tag_pose = [ (None, None) ]
    t1 = threading.Thread(target=video_apriltag_thread, args=(drone, stop_event, latest_tag_pose))
    t2 = threading.Thread(target=pygame_control_thread, args=(drone, stop_event, record_dict, record_lock, lambda: latest_tag_pose[0]))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    drone.end()
    print("Disconnected.")
=======
import cv2
import threading
from djitellopy import Tello
from pupil_apriltags import Detector
import numpy as np
import json
import time
import pygame
from april_tag import calculate_area

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

# Data structure: {tag_id: [list of (translation, rotation matrix)]}
record_dict = {}
record_lock = threading.Lock()

# Helper to get average of list of np arrays

def average_np(arrs):
    arrs = np.array(arrs)
    return np.mean(arrs, axis=0)

# Video/AprilTag/Recording thread

def video_apriltag_thread(drone, stop_event, latest_tag_pose):
    while not stop_event.is_set():
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (960, 720))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray, estimate_tag_pose=True, camera_params=[921.170702,919.018377,459.904354,351.238301], tag_size=0.165)
        tag_id = None
        pose = None
        rot = None
        if results:
            r = results[0]
            tag_id = r.tag_id
            center = (int(r.center[0]), int(r.center[1]))
            corners = r.corners.astype(int)
            area = calculate_area(corners)
            pose_t = r.pose_t.flatten()
            pose_R = r.pose_R
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {tag_id}", (center[0] - 10, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Pos: ({float(pose_t[0]):.2f}, {float(pose_t[1]):.2f}, {float(pose_t[2]):.2f})", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            pose = pose_t
            rot = pose_R
        else:
            pose = None
            rot = None
        latest_tag_pose[0] = (pose, rot, area)
        cv2.imshow("Tello Key Record", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break
    cv2.destroyAllWindows()

def pygame_control_thread(drone, stop_event, record_dict, record_lock, get_latest_tag_pose):
    pygame.init()
    WIDTH, HEIGHT = 400, 200
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Tello Controls & Recording")
    font = pygame.font.SysFont(None, 28)
    speed = 50
    flying = False
    info = ""
    clock = pygame.time.Clock()
    while not stop_event.is_set():
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
                elif key == pygame.K_w:
                    drone.send_rc_control(0, 0, speed, 0)
                    info = "Up"
                elif key == pygame.K_s:
                    drone.send_rc_control(0, 0, -speed, 0)
                    info = "Down"
                elif key == pygame.K_a:
                    drone.send_rc_control(0, 0, 0, -speed)
                    info = "Yaw Left"
                elif key == pygame.K_d:
                    drone.send_rc_control(0, 0, 0, speed)
                    info = "Yaw Right"
                elif key == pygame.K_UP:
                    drone.send_rc_control(0, speed, 0, 0)
                    info = "Forward"
                elif key == pygame.K_DOWN:
                    drone.send_rc_control(0, -speed, 0, 0)
                    info = "Backward"
                elif key == pygame.K_LEFT:
                    drone.send_rc_control(-speed, 0, 0, 0)
                    info = "Left"
                elif key == pygame.K_RIGHT:
                    drone.send_rc_control(speed, 0, 0, 0)
                    info = "Right"
                elif pygame.K_1 <= key <= pygame.K_9:
                    tag_num = key - pygame.K_0
                    latest_pose, latest_rot, latest_area = get_latest_tag_pose()
                    if latest_pose is not None and latest_rot is not None:
                        with record_lock:
                            if tag_num not in record_dict:
                                record_dict[tag_num] = []
                            record_dict[tag_num].append((latest_pose.copy(), latest_rot.copy(), latest_area.copy()))
                            print(f"Recorded tag {tag_num}: pos {latest_pose},\nrot:\n{latest_rot}\narea: {latest_area}")
                            save_records_to_json(record_dict)
                            info = f"Recorded tag {tag_num}"
        # Stop movement if no key is pressed
        keys = pygame.key.get_pressed()
        if not any(keys):
            drone.send_rc_control(0, 0, 0, 0)
        clock.tick(30)
    pygame.quit()

def save_records_to_json(record_dict):
    out = {}
    for tag_id, records in record_dict.items():
        if not records:
            continue
        poses = np.array([r[0] for r in records])
        rots = np.array([r[1] for r in records])
        areas = np.array([r[2] for r in records])
        avg_pose = np.mean(poses, axis=0).tolist()
        avg_rot = np.mean(rots, axis=0).tolist()
        avg_area = np.mean(areas)
        out[tag_id] = {
            "num_records": len(records),
            "avg_translation": avg_pose,
            "avg_rotation_matrix": avg_rot,
            "avg_area": avg_area
        }
    with open("tello_tag_records.json", "w") as f:
        json.dump(out, f, indent=2)

if __name__ == "__main__":
    drone = init_tello()
    stop_event = threading.Event()
    latest_tag_pose = [ (None, None) ]
    t1 = threading.Thread(target=video_apriltag_thread, args=(drone, stop_event, latest_tag_pose))
    t2 = threading.Thread(target=pygame_control_thread, args=(drone, stop_event, record_dict, record_lock, lambda: latest_tag_pose[0]))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    drone.end()
    print("Disconnected.")
>>>>>>> f13d8ee322d8aca2a2066926b09ec2b7f36cd405

from djitellopy import Tello
from time import sleep
import cv2
from pupil_apriltags import Detector
import numpy as np
from movement_func import *
from helper_func import rot2eul

# AprilTag Functions

def calculate_area(corners):
    """Calculate area of quadrilateral formed by tag corners"""
    # Ensure corners are a NumPy array of shape (4, 2) and correct type for contourArea
    return cv2.contourArea(np.array(corners, dtype=np.float32).reshape((-1, 1, 2)))

def calculate_center_offset(center, frame_width, frame_height):
    """Calculate how far tag is from center of frame"""
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2
    return (center[0] - frame_center_x, center[1] - frame_center_y)

def calculate_offset_based_on_center(center, frame_width, frame_height, x, y):
    """Calculate how far tag is from(frame_width // 2 + x, frame_height // 2 + y)"""
    x_position = frame_width // 2 + x
    y_position = frame_height // 2 + y
    
    return (center[0] - x_position, center[1] - y_position)

def detect_apriltag(frame, detector):
    """
    Detect AprilTag in the given frame using the specified detector.
    Returns a list of dictionaries, each containing info for a detected tag (including area).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=[921.170702, 919.018377, 459.904354, 351.238301],
        tag_size=0.092
    )
    processed_tags = []
    if detections:
        for r in detections:  # r is a pupil_apriltags.Detection object
            tag_id = r.tag_id
            center = (int(r.center[0]), int(r.center[1])) # Convert center to tuple of ints
            corners = r.corners  # NumPy array of shape (4, 2), type float
            pose_t = r.pose_t.flatten()
            pose_R = r.pose_R
            angle = rot2eul(pose_R)
            area = calculate_area(corners)  # Calculate area

            processed_tags.append(
                {
                    'tag_id': tag_id,
                    'center': center,
                    'corners': corners,
                    'pose_t': pose_t,
                    'pose_R': pose_R,
                    'angle': angle,
                    'area': area
                }
            )
    print("Tags: ", processed_tags)
    return processed_tags # Return the list of processed dictionaries

def draw_apriltag_info(frame, tag_data, frame_width, frame_height, target_x_offset_px=None, target_y_offset_px=None, target_area=None):
    """Draws AprilTag detection information on the frame."""
    if not tag_data:
        print("No tags to draw")
        return

    center = tag_data['center'] # tuple of ints
    corners = tag_data['corners'].astype(int) # Convert float corners to int for drawing
    angle = tag_data.get('angle', [0,0,0])
    pose_t = tag_data.get('pose_t', [0,0,0])
    tag_id = tag_data.get('tag_id', -1)
    area_display = tag_data.get('area', 0)

    offset_x_display, offset_y_display = calculate_center_offset(center, frame_width, frame_height)

    print(f"Drawing Tag {tag_id}")

    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
    cv2.circle(frame, center, 5, (0, 0, 255), -1)
    cv2.putText(frame, f"ID: {tag_id}", (center[0] - 10, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if target_area:
        cv2.putText(frame, f"Area: {area_display:.0f} (Tgt: {target_area})", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    else:
        cv2.putText(frame, f"Area: {area_display:.0f}", (center[0] - 10, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, f"Angle(yaw): {angle[1]:.2f} rad", (center[0] - 10, center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, f"Pose t: ({pose_t[0]:.2f}, {pose_t[1]:.2f}, {pose_t[2]:.2f})m", (center[0] - 10, center[1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),2)
    if target_x_offset_px:
        cv2.putText(frame, f"CamOffsetPx: ({offset_x_display:.0f},{offset_y_display:.0f}) TgtOffsetPx: ({target_x_offset_px},{target_y_offset_px})", (center[0] - 10, center[1] + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),2)
    else:
        cv2.putText(frame, f"CamOffsetPx: ({offset_x_display:.0f},{offset_y_display:.0f})", (center[0] - 10, center[1] + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),2)

    return frame
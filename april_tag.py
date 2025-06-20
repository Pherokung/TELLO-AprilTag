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
    return cv2.contourArea(corners.reshape((-1, 1, 2)))

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

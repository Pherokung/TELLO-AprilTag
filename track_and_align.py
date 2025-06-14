import numpy as np
AREA_TOLERANCE = 12000

pid_yaw = [0.4, 0.4, 0]
pid_ud = [0.2, 0.15, 0]

def is_aligning_done(angle, angle_threshold):
    if abs(angle[1]) < angle_threshold:
        return True

def aligning_angle(tello, angle, angle_threshold):

    if is_aligning_done(angle, angle_threshold):
        tello.send_rc_control(0, 0, 0, 0)
        print(f"Aligned! ")
        return True
    else:
        # Control logic
        if angle[1] >= angle_threshold:
            yaw_speed = 30
            lr = -25
            
        elif angle[1] <= -angle_threshold:
            yaw_speed = -30
            lr = 25
            
        else:
            yaw_speed = 0
        
        tello.send_rc_control(lr, 0, 0, yaw_speed)
        return False
    
def is_tracking_done(area, target_area, error_yaw, error_ud):
    if area > target_area - AREA_TOLERANCE and area < target_area + AREA_TOLERANCE:
        error_yaw = int(np.clip(pid_yaw[0] * error_yaw, -100, 100))
        error_ud = int(np.clip(pid_ud[0] * error_ud, -100, 100))
        if abs(error_yaw) < 10 and abs(error_ud) < 10:
            return True
    return False
       

def track_apriltag(tello, area, 
                   ox, oy, 
                   pError_yaw, pError_ud, 
                   target_area):

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
    
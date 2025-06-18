from djitellopy import Tello
from time import sleep
import math

# Tello Movement Functions

def move_tello(drone, direction, distance, speed = 75):
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
    return time

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

    sleep(total_time)
    return total_time


def move_tello_bearing(drone, bearing, distance, speed = 75):
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

    print(f"Moving {distance}cm at {bearing}deg")

    radians = math.radians(bearing)
    x = int(distance * math.cos(radians))
    y = int(distance * math.sin(radians))
    print(f"Moving with {y} lr and {x} fb")

    drone.send_rc_control(y, x, 0, 0)
    print(f"Sleep for {distance / speed}")
    # sleep(1)
    sleep(distance / speed)
    return distance / speed

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
    
    sleep(dist / speed)
    return dist / speed 
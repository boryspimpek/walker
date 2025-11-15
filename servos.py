import math
from ik import solve_ik_2d
from config import L1, L2

from st3215 import ST3215
servo = ST3215('COM3')

acc = 250
speed = 4000
sts_id = [2, 3, 4]   
angle_limits = {2: (30, 150), 3: (50, 160), 4: (80, 100)}
trims = {2: -145, 3: -145, 4: 10}

def check_angle_limit(id, angle_deg):
    min_angle, max_angle = angle_limits.get(id, (-180, 180))
    if angle_deg < min_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° poniżej minimum ({min_angle}°) — ograniczono.")
        angle_deg = min_angle
    elif angle_deg > max_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° powyżej maksimum ({max_angle}°) — ograniczono.")
        angle_deg = max_angle
    return angle_deg

def PrepareMove():
    for id in sts_id:
        try:
            servo.SetMode(id, 0)
            servo.SetAcceleration(id, acc)
            servo.SetSpeed(id, speed)
        except Exception as e:
            print(f"Error initializing servo {id}: {e}")

def MoveServo(id, angle_deg):
    try:
        safe_angle = check_angle_limit(id, angle_deg)
        pos = servo.angle_deg_to_servo(safe_angle)
        trimmed_pos = pos + trims.get(id, 0)
        servo.WritePosition(id, trimmed_pos)
    except Exception as e:
        print(f"Error moving servo {id}: {e}")

def ReturnToNeutral():
    neutral_positions = {2: 90, 3: 90, 4: 90}
    for id, angle in neutral_positions.items():
        MoveServo(id, angle)

def MoveToPoint(x, z):
    ik = solve_ik_2d(x, z, L1, L2)
    if ik is not None:
        t1, t2 = ik
        deg1 = math.degrees(t1)
        deg2 = math.degrees(t2)
        deg3 = deg1 + deg2
        MoveServo(2, deg1)
        MoveServo(3, deg3)
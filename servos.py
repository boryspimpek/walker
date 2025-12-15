import math
import time
from ik import solve_ik_2d
from config import L1, L2, sts_id, MAX_SPEED, ACC, angle_limits, trims

from st3215 import ST3215
servo = ST3215('COM3')


def check_angle_limit(id, angle_deg):
    min_angle, max_angle = angle_limits.get(id, (-180, 180))
    if angle_deg < min_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° poniżej minimum ({min_angle}°) — ograniczono.")
        angle_deg = min_angle
    elif angle_deg > max_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° powyżej maksimum ({max_angle}°) — ograniczono.")
        angle_deg = max_angle
    return angle_deg

def MoveServo(id, angle_deg, speed=None, acc=None):
    if speed is None:
        speed = MAX_SPEED
    if acc is None:
        acc = ACC
    try:
        safe_angle = check_angle_limit(id, angle_deg)
        pos = servo.angle_deg_to_servo(safe_angle)
        trimmed_pos = pos + trims.get(id, 0)

        servo.SetMode(id, 0)
        servo.SetAcceleration(id, acc)
        servo.SetSpeed(id, speed)

        servo.WritePosition(id, trimmed_pos)
    except Exception as e:
        print(f"Error moving servo {id}: {e}")

def MoveToPoint(x, z, leg, speed=None, acc=None):
    if speed is None:
        speed = MAX_SPEED
    if acc is None:
        acc = ACC

    if leg == 'right':
        ik = solve_ik_2d(-x, z, L1, L2, elbow_up=False)
        if ik is not None:
            t1, t2 = ik
            MoveServo(2, math.degrees(t1), speed, acc)
            MoveServo(3, math.degrees(t1) + math.degrees(t2), speed, acc)
    else:
        ik = solve_ik_2d(x, z, L1, L2, elbow_up=True)
        if ik is not None:
            t1, t2 = ik
            MoveServo(6, math.degrees(t1), speed, acc)
            MoveServo(7, math.degrees(t1) + math.degrees(t2), speed, acc)

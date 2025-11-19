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

def MoveSync(target_deg, speed=MAX_SPEED, acc=ACC):
    safe_target_deg = {}
    for id in sts_id:
        try:
            safe_target_deg[id] = check_angle_limit(id, target_deg[id])
        except Exception as e:
            return

    curr_deg = {}
    for id in sts_id:
        unit = servo.ReadPosition(id)
        if unit is None:
            # print(f"❌ Error: cannot read position of servo {id}")
            return

        unit_corrected = unit - trims.get(id, 0)
        deg = servo.servo_to_deg(unit_corrected)
        curr_deg[id] = deg

    distances = {
        id: abs(safe_target_deg[id] - curr_deg[id])
        for id in sts_id}
    
    max_distance = max(distances.values())
    if max_distance == 0:
        print("➡ No movement required. Exiting.")
        return

    for id in sts_id:
        dist = distances[id]

        if dist == 0:
            corected_speed = 0
        else:
            corected_speed = int((dist / max_distance) * speed * 0.5)
            if corected_speed < 50:
                corected_speed = 50

        target_unit = servo.angle_deg_to_servo(safe_target_deg[id])
        trimmed_pos = target_unit + trims.get(id, 0)

        servo.SetMode(id, 0)
        servo.SetAcceleration(id, acc)
        servo.SetSpeed(id, speed)

        servo.WritePosition(id, trimmed_pos)

def MoveServo(id, angle_deg, speed=MAX_SPEED, acc=ACC):
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

def ReturnToNeutral():
    neutral_positions = {1: 90, 2: 52.8, 3: 108.3, 4: 90, 5: 90, 6: 127.2, 7: 71.7, 8: 90}
    for id, angle in neutral_positions.items():
        MoveServo(id, angle, 500, 50)

def MoveToPoint(x, z, leg, speed=MAX_SPEED, acc=ACC):
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
            print(speed, acc)
            MoveServo(6, math.degrees(t1), speed, acc)
            MoveServo(7, math.degrees(t1) + math.degrees(t2), speed, acc)

def steps(speed, acc):
    frame0 = {1: 90, 2: 90, 3: 90, 4: 90, 5: 90, 6: 90, 7: 90, 8: 90}
    frame1 = {1: 95, 2: 90, 3: 90, 4: 105, 5: 90, 6: 90, 7: 90, 8: 105}
    frame3 = {1: 90, 2: 90, 3: 90, 4: 75, 5: 85, 6: 90, 7: 90, 8: 75}


    while True:
        for id in sts_id:
            MoveSync(frame1, speed, acc)
        time.sleep(0.4)

        MoveToPoint(0, -80, "left", speed, acc)
        time.sleep(0.9)
        MoveToPoint(0, -110, "left", speed, acc)
        time.sleep(0.9)

        for id in sts_id:
            MoveSync(frame0, speed, acc)
        time.sleep(0.5)

        for id in sts_id:
            MoveSync(frame3, speed, acc)
        time.sleep(0.5)

        MoveToPoint(0, -80, "right", speed, acc)
        time.sleep(0.9)
        MoveToPoint(0, -110, "right", speed, acc)
        time.sleep(0.9)


        for id in sts_id:
            MoveSync(frame0, speed, acc)
        time.sleep(0.5)

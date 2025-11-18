import math
import time
from ik import solve_ik_2d
from config import L1, L2, sts_id, MAX_SPEED, MIN_SPEED, acc, angle_limits, trims

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

def MoveSync(target_deg):
    print("\n========== MoveSync START ==========\n")

    print("▶ Target angles (deg):", target_deg)

    # 1. Sprawdzamy limity
    safe_target_deg = {}
    print("\n--- Checking angle limits ---")
    for id in sts_id:
        try:
            safe_target_deg[id] = check_angle_limit(id, target_deg[id])
            print(f"Servo {id}: target={target_deg[id]}° → safe={safe_target_deg[id]}°")
        except Exception as e:
            print(f"❌ Angle limit error on servo {id}: {e}")
            return

    # 2. Odczyt pozycji bieżącej w UNIT → DEG, uwzględniając trim
    curr_deg = {}
    print("\n--- Reading current positions ---")
    for id in sts_id:
        unit = servo.ReadPosition(id)
        if unit is None:
            print(f"❌ Error: cannot read position of servo {id}")
            return

        # Odejmujemy trim (w jednostkach serwa)
        unit_corrected = unit - trims.get(id, 0)

        deg = servo.servo_to_deg(unit_corrected)
        curr_deg[id] = deg
        print(f"Servo {id}: raw_unit={unit}, corrected_unit={unit_corrected}, deg={deg}")

    # 3. Obliczamy dystanse
    print("\n--- Calculating distances ---")
    distances = {
        id: abs(safe_target_deg[id] - curr_deg[id])
        for id in sts_id
    }
    for id in sts_id:
        print(f"Servo {id}: distance = {distances[id]}°")

    max_distance = max(distances.values())
    print(f"\nMAX distance = {max_distance}°")

    if max_distance == 0:
        print("➡ No movement required. Exiting.")
        return

    # 4. Ustawiamy prędkości i targety
    print("\n--- Setting speeds and writing positions ---")
    for id in sts_id:
        dist = distances[id]

        if dist == 0:
            speed = 0
        else:
            speed = int((dist / max_distance) * MAX_SPEED * 0.5)
            if speed < MIN_SPEED:
                speed = MIN_SPEED

        print(f"Servo {id}: dist={dist}° → speed={speed}")

        # Konwersja do unit
        target_unit = servo.angle_deg_to_servo(safe_target_deg[id])
        trimmed_pos = target_unit + trims.get(id, 0)

        print(f"Servo {id}: safe target={safe_target_deg[id]}°, trimmed unit={trimmed_pos}")

        servo.SetMode(id, 0)
        servo.SetAcceleration(id, acc)
        servo.SetSpeed(id, speed)

        servo.WritePosition(id, trimmed_pos)

    print("\n========== MoveSync END ==========\n")

def MoveServo(id, angle_deg):
    try:
        safe_angle = check_angle_limit(id, angle_deg)
        pos = servo.angle_deg_to_servo(safe_angle)
        trimmed_pos = pos + trims.get(id, 0)

        servo.SetMode(id, 0)
        servo.SetAcceleration(id, acc)
        servo.SetSpeed(id, MAX_SPEED)
        
        servo.WritePosition(id, trimmed_pos)
    except Exception as e:
        print(f"Error moving servo {id}: {e}")

def ReturnToNeutral():
    neutral_positions = {1: 90, 2: 90, 3: 90, 4: 90, 5: 90, 6: 90, 7: 90, 8: 90}
    for id, angle in neutral_positions.items():
        MoveServo(id, angle)

def MoveToPoint(x, z, leg):
    if leg == 'right':
        ik = solve_ik_2d(-x, z, L1, L2, elbow_up=False)
        if ik is not None:
            t1, t2 = ik
            print(f"IK Solution for (x={x}, z={z}): deg1 = {math.degrees(t1)}°, deg3 = {math.degrees(t1) + math.degrees(t2)}°")
            MoveServo(2, math.degrees(t1))
            MoveServo(3, math.degrees(t1) + math.degrees(t2))
    else:
        ik = solve_ik_2d(x, z, L1, L2, elbow_up=True)
        if ik is not None:
            t1, t2 = ik
            print(f"IK Solution for (x={x}, z={z}): deg1 = {math.degrees(t1)}°, deg3 = {math.degrees(t1) + math.degrees(t2)}°")
            MoveServo(6, math.degrees(t1))
            MoveServo(7, math.degrees(t1) + math.degrees(t2))

def steps():
    frame0 = {1: 90, 2: 90, 3: 90, 4: 90, 5: 90, 6: 90, 7: 90, 8: 90}
    frame1 = {1: 95, 2: 90, 3: 90, 4: 105, 5: 90, 6: 90, 7: 90, 8: 105}
    frame3 = {1: 90, 2: 90, 3: 90, 4: 75, 5: 85, 6: 90, 7: 90, 8: 75}


    while True:
        for id in sts_id:
            MoveSync(frame1)
        time.sleep(0.5)

        MoveToPoint(0, -80, "left")
        time.sleep(1)
        MoveToPoint(0, -110, "left")
        time.sleep(1)

        for id in sts_id:
            MoveSync(frame0)
        time.sleep(0.6)

        for id in sts_id:
            MoveSync(frame3)
        time.sleep(0.5)

        MoveToPoint(0, -80, "right")
        time.sleep(1)
        MoveToPoint(0, -110, "right")
        time.sleep(1)


        for id in sts_id:
            MoveSync(frame0)
        time.sleep(0.6)

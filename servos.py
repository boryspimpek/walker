from st3215 import ST3215
servo = ST3215('COM3')


acc = 250
speed = 4000
sts_id = [2, 3]   
angle_limits = {2: (30, 150), 3: (60, 160)}
trims = {2: -200, 3: -230}

def check_angle_limit(id, angle_deg):
    min_angle, max_angle = angle_limits.get(id, (-180, 180))
    if angle_deg < min_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° poniżej minimum ({min_angle}°) — ograniczono.")
        angle_deg = min_angle
    elif angle_deg > max_angle:
        print(f"⚠️ Servo {id}: kąt {angle_deg}° powyżej maksimum ({max_angle}°) — ograniczono.")
        angle_deg = max_angle
    return angle_deg

def prepare_move():
    for id in sts_id:
        try:
            servo.SetMode(id, 0)
            servo.SetAcceleration(id, acc)
            servo.SetSpeed(id, speed)
        except Exception as e:
            print(f"Error initializing servo {id}: {e}")

def move_servo(id, angle_deg):
    try:
        safe_angle = check_angle_limit(id, angle_deg)
        pos = servo.angle_deg_to_servo(safe_angle)
        trimmed_pos = pos + trims.get(id, 0)
        servo.WritePosition(id, trimmed_pos)
    except Exception as e:
        print(f"Error moving servo {id}: {e}")

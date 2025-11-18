import keyboard  
import time
from config import angle_limits, trims, sts_id 
from servos import MoveServo

servo_keys = {
    1: ('q', 'a'),
    2: ('w', 's'),
    3: ('e', 'd'),
    4: ('r', 'f'),
    5: ('t', 'g'),
    6: ('y', 'h'),
    7: ('u', 'j'),
    8: ('i', 'k'),
}

STEP = 2  # krok ruchu
positions = {sid: 90 for sid in sts_id}
initial_positions = positions.copy()

def center_all_servos():
    print("Ustawiam wszystkie serwa na 90")
    for sid in sts_id:
        MoveServo(sid, 90)
        time.sleep(0.5)
    print("Gotowe! Wszystkie serwa są w pozycji neutralnej.\n")

def move():
    print("Sterowanie:")
    for sid, (left, right) in servo_keys.items():
        print(f"Serwo {sid}: {left} / {right}")
    print("Naciśnij 'x', aby zakończyć.\n")

    while True:
        for sid, (left, right) in servo_keys.items():
            if keyboard.is_pressed(left):
                positions[sid] = max(angle_limits[sid][0], positions[sid] - STEP)
                MoveServo(sid, positions[sid])
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

            elif keyboard.is_pressed(right):
                positions[sid] = min(angle_limits[sid][1], positions[sid] + STEP)
                MoveServo(sid, positions[sid])
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

        if keyboard.is_pressed('x'):
            print("\nZakończono.\n")
            break

    print("📋 Podsumowanie:")
    for sid in sts_id:
        print(f"Serwo {sid} (końcowa pozycja: {positions[sid]})")


if __name__ == "__main__":
    center_all_servos()
    move()

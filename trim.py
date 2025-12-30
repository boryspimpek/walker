import keyboard
from st3215 import ST3215
import time
from config import trims, sts_id

servo = ST3215('COM3')

BASE_POSITION = 1024

positions = {sid: BASE_POSITION + trims.get(sid, 0) for sid in sts_id}
initial_positions = positions.copy()

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

STEP = 5  
SPEED = 200
ACC_LOCAL = 10  # lokalne do trybu trimowania


def center_all_servos():
    print("Ustawiam wszystkie serwa na pozycje z trimów...")
    for sid in sts_id:
        servo.MoveTo(sid, positions[sid], SPEED, ACC_LOCAL, False)
        time.sleep(0.05)
    print("Gotowe! Serwa ustawione wg zapisanych trimów.\n")


def trim_servos():
    print("Tryb trimowania uruchomiony.")
    print("Sterowanie:")
    for sid, (left, right) in servo_keys.items():
        print(f"Serwo {sid}: {left} / {right}")
    print("Naciśnij 'x', aby zakończyć.\n")

    while True:
        for sid, (left, right) in servo_keys.items():
            if keyboard.is_pressed(left):
                positions[sid] = max(0, positions[sid] - STEP)
                servo.MoveTo(sid, positions[sid], SPEED, ACC_LOCAL, False)
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

            elif keyboard.is_pressed(right):
                positions[sid] = min(2048, positions[sid] + STEP)
                servo.MoveTo(sid, positions[sid], SPEED, ACC_LOCAL, False)
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

        if keyboard.is_pressed('x'):
            print("\nZakończono trimowanie.\n")
            break

    print("📋 Podsumowanie zmian względem zapisanych trimów:")
    for sid in sts_id:
        diff = positions[sid] - (BASE_POSITION + trims.get(sid, 0))
        znak = "+" if diff > 0 else ""
        print(f"Serwo {sid} -> {znak}{diff} (końcowa pozycja: {positions[sid]})")


if __name__ == "__main__":
    center_all_servos()
    trim_servos()

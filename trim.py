import keyboard  # pip install keyboard
from st3215 import ST3215
import time

servo = ST3215('COM3')

# Lista ID serw
sts_id = [1, 2, 3, 4, 5, 6, 7, 8]

# Pozycje startowe wszystkich serw
positions = {sid: 1024 for sid in sts_id}
initial_positions = positions.copy()

# Mapowanie klawiszy do serw
# Każde serwo ma dwa klawisze: (w lewo, w prawo)
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

STEP = 5  # krok ruchu
SPEED = 200
ACC = 10


def center_all_servos():
    print("Ustawiam wszystkie serwa na 1024...")
    for sid in sts_id:
        servo.MoveTo(sid, 1024, SPEED, ACC, False)
        time.sleep(0.05)
    print("Gotowe! Wszystkie serwa są w pozycji neutralnej.\n")


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
                servo.MoveTo(sid, positions[sid], SPEED, ACC, False)
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

            elif keyboard.is_pressed(right):
                positions[sid] = min(2048, positions[sid] + STEP)
                servo.MoveTo(sid, positions[sid], SPEED, ACC, False)
                print(f"Serwo {sid}: {positions[sid]}")
                time.sleep(0.1)

        if keyboard.is_pressed('x'):
            print("\nZakończono trimowanie.\n")
            break

    print("📋 Podsumowanie zmian:")
    for sid in sts_id:
        diff = positions[sid] - initial_positions[sid]
        znak = "+" if diff > 0 else ""
        print(f"Serwo {sid} -> {znak}{diff} (końcowa pozycja: {positions[sid]})")


if __name__ == "__main__":
    center_all_servos()
    trim_servos()

import math
import keyboard
import time

from ik import solve_ik_2d
from servos import PrepareMove, MoveServo
from config import L1, L2

def control(X=0, Z=-110, STEP=2):
    print("Sterowanie końcówką:")
    print(" W/S = Z+/Z-")
    print(" A/D = X-/X+")
    print(" X = wyjście")
    print()

    while True:
        moved = False

        if keyboard.is_pressed("s"):
            Z += STEP
            moved = True
        if keyboard.is_pressed("w"):
            Z -= STEP
            moved = True
        if keyboard.is_pressed("d"):
            X -= STEP
            moved = True
        if keyboard.is_pressed("a"):
            X += STEP
            moved = True

        if moved:
            ik = solve_ik_2d(X, Z, L1, L2, elbow_up=True)

            if ik is None:
                print("Poza zasięgiem!", X, Z)
            else:
                t1, t2 = ik
                deg1 = math.degrees(t1)
                deg2 = math.degrees(t2)
                deg3 = deg1 + deg2
                MoveServo(2, deg1)
                MoveServo(3, deg3)   

                print(f"X={X} Z={Z} | t1={deg1:.1f} t3={deg3:.1f}")

            time.sleep(0.05)

if __name__ == "__main__":
    PrepareMove()
    control()
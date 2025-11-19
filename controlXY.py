import math
import keyboard
import time

from ik import solve_ik_2d
from servos import MoveServo
from config import L1, L2

def control(X=0, Z=-110, STEP=1):
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
            ik_l = solve_ik_2d(X, Z, L1, L2, elbow_up=False)
            ik_r = solve_ik_2d(-X, Z, L1, L2, elbow_up=True)

            if ik_l is None or ik_r is None:
                print("Poza zasięgiem!", X, Z)            
            
            else:
                t1, t2 = ik_l
                deg1 = math.degrees(t1)
                deg2 = math.degrees(t2)
                deg3 = deg1 + deg2
                MoveServo(2, deg1)
                MoveServo(3, deg3)

                t3, t4 = ik_r
                deg4 = math.degrees(t3)
                deg5 = math.degrees(t4)
                deg6 = deg4 + deg5
                MoveServo(6, deg4)
                MoveServo(7, deg6)   

                print(f"X={X} Z={Z} | servo2={deg1:.1f} servo3={deg3:.1f} | servo6={deg4:.1f} servo7={deg6:.1f}")

            time.sleep(0.05)

if __name__ == "__main__":
    control()
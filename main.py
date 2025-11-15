import math
import keyboard
import time
import numpy as np

from ik import solve_ik_2d
from servos import PrepareMove, MoveServo, ReturnToNeutral, MoveToPoint
from config import L1, L2, CYCLE_TIME, SWING_WIDTH, SWING_HEIGHT, X_OFFSET, BASE_Z, T_POINTS

def trot_gait(phase):
    half_width = SWING_WIDTH / 2

    if phase < 0.5:
        angle = math.pi * (1 - 2 * phase)
        x = half_width * math.cos(angle) + X_OFFSET
        z = BASE_Z + SWING_HEIGHT * math.sin(angle)
    else:
        t = (phase - 0.5) * 2
        x = half_width - SWING_WIDTH * t + X_OFFSET
        z = BASE_Z

    return x, z

def runTrotGait():
    start = time.time()

    while True:
        now = time.time()
        dt = now - start
        phase = (dt % CYCLE_TIME) / CYCLE_TIME

        x, z = trot_gait(phase)

        ik = solve_ik_2d(x, z, L1, L2, elbow_up=False)
        if ik is not None:
            t1, t2 = ik
            deg1 = math.degrees(t1)
            deg2 = math.degrees(t2)
            deg3 = deg1 + deg2

            MoveServo(2, deg1)
            MoveServo(3, deg3)

        # offset 90°, amplituda 15°
        # tilt_angle = 90 + 10 * math.sin(2 * math.pi * phase - math.pi/2)
        # moveServo(4, tilt_angle)

        time.sleep(0.02)

if __name__ == "__main__":
    PrepareMove()



    runTrotGait()
    # ReturnToNeutral()
    # MoveToPoint(0, -110)
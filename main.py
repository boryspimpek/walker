import math
import time
import numpy as np

from ik import solve_ik_2d
from servos import MoveServo, ReturnToNeutral, MoveToPoint, steps
from config import L1, L2, CYCLE_TIME, SWING_WIDTH, SWING_HEIGHT, X_OFFSET, BASE_Z, MAX_TILT

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

def tilt(phase):
    """
    Dwukierunkowy rounded square wave (tam i z powrotem).
    phase ∈ [0, 1]
    """
    MID_1 = 0.25
    MID_2 = 0.5
    MID_3 = 0.75

    if phase < MID_1:
        # stałe LEWO
        return 90 - MAX_TILT

    elif phase < MID_2:
        # przejazd LEWO -> PRAWO
        t = (phase - MID_1) / (MID_2 - MID_1)
        t_smooth = 3*t*t - 2*t*t*t
        return (90 - MAX_TILT) + 2 * MAX_TILT * t_smooth

    elif phase < MID_3:
        # stałe PRAWO
        return 90 + MAX_TILT

    else:
        # przejazd PRAWO -> LEWO
        t = (phase - MID_3) / (MID_3 - MID_2)
        t_smooth = 3*t*t - 2*t*t*t
        return (90 + MAX_TILT) - 2 * MAX_TILT * t_smooth

def runTrotGaitTwoLegs():
    """
    Dwunożny trot z naprzemiennym krokiem i tilt/roll.
    """
    start = time.perf_counter()

    while True:
        now = time.perf_counter()
        dt = now - start
        phase = (dt % CYCLE_TIME) / CYCLE_TIME  # normalizacja do [0,1)

        # ====== Fazy nóg ======
        phase_right = phase
        phase_left  = (phase + 0.5) % 1.0  # 180° przesunięcie

        # ====== Pozycje stóp ======
        x_r, z_r = trot_gait(phase_right)
        x_l, z_l = trot_gait(phase_left)

        # ====== IK dla prawej nogi ======
        ik_r = solve_ik_2d(-x_r, z_r, L1, L2, elbow_up=False)
        if ik_r is not None:
            t1_r, t2_r = ik_r
            deg1_r = math.degrees(t1_r)
            deg2_r = math.degrees(t2_r)
            deg3_r = deg1_r + deg2_r
            MoveServo(2, deg1_r)  
            MoveServo(3, deg3_r)  

        # ====== IK dla lewej nogi ======
        ik_l = solve_ik_2d(x_l, z_l, L1, L2, elbow_up=True)
        if ik_l is not None:
            t1_l, t2_l = ik_l
            deg1_l = math.degrees(t1_l)
            deg2_l = math.degrees(t2_l)
            deg3_l = deg1_l + deg2_l
            MoveServo(6, deg1_l)  
            MoveServo(7, deg3_l)  

        # ====== Tilt serwa (CoM) ======
        tilt_angle = tilt(phase)  # tilt zsynchronizowany z cyklem głównym
        MoveServo(4, tilt_angle)
        MoveServo(8, tilt_angle)
        MoveServo(1, tilt_angle)
        MoveServo(5, tilt_angle)

        # ====== Opóźnienie ======
        time.sleep(0.02)

if __name__ == "__main__":
    try:
        # PrepareMove()


        # runTrotGaitTwoLegs()
        steps()
        # MoveToPoint(0, -80)
        # time.sleep(5)
        # MoveToPoint(0, -110)
        # time.sleep(5)
    
    except KeyboardInterrupt:
        print("\nPrzerwano program przez Ctrl+C")
    finally:
        ReturnToNeutral()
        print("Powrót do pozycji neutralnej zakończony")

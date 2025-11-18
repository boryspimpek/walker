import math
import time

from ik import solve_ik_2d
from servos import MoveServo, ReturnToNeutral, MoveToPoint, steps
from config import L1, L2, CYCLE_TIME, SWING_WIDTH, SWING_HEIGHT, X_OFFSET, BASE_Z, FOOT_TILT, HIP_TILT

SWING_TIME = 0.35

def trot_gait(phase):
    half_width = SWING_WIDTH / 2
    
    if phase < SWING_TIME:
        # Normalizuj phase do [0,1] w fazie swing
        t_swing = phase / SWING_TIME
        angle = math.pi * (1 - t_swing)
        x = -half_width * math.cos(angle) + X_OFFSET
        z = BASE_Z + SWING_HEIGHT * math.sin(angle)
    else:
        # Normalizuj phase do [0,1] w fazie support
        t_support = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = -half_width + SWING_WIDTH * t_support + X_OFFSET
        z = BASE_Z
    
    return x, z

def tilt(phase):
    if phase < SWING_TIME:
        # stałe LEWO
        left_foot_tilt = 90 - FOOT_TILT
        right_foot_tilt = 90 - FOOT_TILT
        left_hip_tilt = 90 - HIP_TILT
        right_hip_tilt = 90
        print(f"{left_foot_tilt:.0f} {right_foot_tilt:.0f} {left_hip_tilt:.0f} {right_hip_tilt:.0f}")
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    elif phase < 0.5:
        # przejazd LEWO -> PRAWO
        progress = (phase - SWING_TIME) / (0.5 - SWING_TIME)  # 0 to 1
        left_foot_tilt = 90 - FOOT_TILT + (2 * FOOT_TILT * progress)
        right_foot_tilt = 90 - FOOT_TILT + (2 * FOOT_TILT * progress)
        left_hip_tilt = 90 - HIP_TILT + (HIP_TILT * progress)
        right_hip_tilt = 90 + (HIP_TILT * progress)
        print(f"{left_foot_tilt:.0f} {right_foot_tilt:.0f} {left_hip_tilt:.0f} {right_hip_tilt:.0f}")
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    elif phase < 0.5 + SWING_TIME:
        # stałe PRAWO
        left_foot_tilt = 90 + FOOT_TILT
        right_foot_tilt = 90 + FOOT_TILT
        left_hip_tilt = 90
        right_hip_tilt = 90 + HIP_TILT
        print(f"{left_foot_tilt:.0f} {right_foot_tilt:.0f} {left_hip_tilt:.0f} {right_hip_tilt:.0f}")
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    else:
        # przejazd PRAWO -> LEWO
        progress = (phase - (0.5 + SWING_TIME)) / (1.0 - (0.5 + SWING_TIME))  # 0 to 1
        left_foot_tilt = 90 + FOOT_TILT - (2 * FOOT_TILT * progress)
        right_foot_tilt = 90 + FOOT_TILT - (2 * FOOT_TILT * progress)
        left_hip_tilt = 90 - (HIP_TILT * progress)
        right_hip_tilt = 90 + HIP_TILT - (HIP_TILT * progress)
        print(f"{left_foot_tilt:.0f} {right_foot_tilt:.0f} {left_hip_tilt:.0f} {right_hip_tilt:.0f}")
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt
            
def runTrotGaitTwoLegs():
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
        left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt = tilt(phase)
        
        # Apply tilt servos
        MoveServo(8, left_foot_tilt)
        MoveServo(4, right_foot_tilt)
        MoveServo(5, left_hip_tilt)
        MoveServo(1, right_hip_tilt)

        # ====== Opóźnienie ======
        time.sleep(0.02)

if __name__ == "__main__":
    try:
        runTrotGaitTwoLegs()
    
    except KeyboardInterrupt:
        print("\nPrzerwano program przez Ctrl+C")
    finally:
        ReturnToNeutral()
        print("Powrót do pozycji neutralnej zakończony")

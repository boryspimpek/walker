import math
import time

from ik import solve_ik_2d
from servos import MoveServo
from config import HOLD_TIME, L1, L2, CYCLE_TIME, LEFT_FOOT_TILT, LEFT_HIP_TILT, RIGHT_FOOT_TILT, RIGHT_HIP_TILT, SWING_WIDTH, SWING_HEIGHT, SWING_TIME, X_OFFSET, BASE_Z

def trot_gait(phase):
    half_width = SWING_WIDTH / 2
    
    if phase < SWING_TIME:
        t_swing = phase / SWING_TIME
        angle = math.pi * (1 - t_swing)
        x = -half_width * math.cos(angle) + X_OFFSET
        z = BASE_Z + SWING_HEIGHT * math.sin(angle)
    else:
        t_support = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = -half_width + SWING_WIDTH * t_support + X_OFFSET
        z = BASE_Z
    return x, z

def tilt(phase):
    # Normalizacja fazy na zakres 0–1 (opcjonalnie – jeśli phase może być spoza zakresu)
    phase = phase % 1.0

    # Wyznaczenie granic
    p1 = HOLD_TIME               # Przechył w prawo
    p2 = 0.5                     # Koniec jazdy prawo → lewo
    p3 = 0.5 + HOLD_TIME         # Przechył w lewo
    # p4 = 1.0                   # Koniec jazdy lewo → prawo

    # --- FAZA 1: stałe przechylenie w prawo ---
    if phase < p1:
        return (
            90 - LEFT_FOOT_TILT,
            90 - RIGHT_FOOT_TILT,
            90,
            90 - RIGHT_HIP_TILT
        )

    # --- FAZA 2: przejazd prawo → lewo ---
    elif phase < p2:
        progress = (phase - p1) / (p2 - p1)
        return (
            90 - LEFT_FOOT_TILT  + 2 * LEFT_FOOT_TILT * progress,
            90 - RIGHT_FOOT_TILT + 2 * RIGHT_FOOT_TILT * progress,
            90 + LEFT_HIP_TILT * progress,
            90 - RIGHT_HIP_TILT + RIGHT_HIP_TILT * progress
        )

    # --- FAZA 3: stałe przechylenie w lewo ---
    elif phase < p3:
        return (
            90 + LEFT_FOOT_TILT,
            90 + RIGHT_FOOT_TILT,
            90 + LEFT_HIP_TILT,
            90
        )

    # --- FAZA 4: przejazd lewo → prawo ---
    else:
        progress = (phase - p3) / (1.0 - p3)
        return (
            90 + LEFT_FOOT_TILT  - 2 * LEFT_FOOT_TILT * progress,
            90 + RIGHT_FOOT_TILT - 2 * RIGHT_FOOT_TILT * progress,
            90 + LEFT_HIP_TILT   - LEFT_HIP_TILT * progress,
            90 - RIGHT_HIP_TILT * progress
        )
            
def runTrotGaitTwoLegs(num_cycles):
    print(f"Rozpoczynanie chodu trot gait na {num_cycles} cykli...")
    start = time.perf_counter()
    end_time = start + (num_cycles * CYCLE_TIME)  # Czas zakończenia

    while time.perf_counter() < end_time:
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
        MoveServo(4, left_foot_tilt)
        MoveServo(8, right_foot_tilt)
        MoveServo(1, left_hip_tilt)
        MoveServo(5, right_hip_tilt)

        # ====== Opóźnienie ======
        time.sleep(0.02)
    
    print(f"Zakończono {num_cycles} cykle chodu")

def WalkForward(num_cycles):
    runTrotGaitTwoLegs(num_cycles)

WalkForward(10)
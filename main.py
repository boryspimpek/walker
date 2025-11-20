import math
import time

from ik import solve_ik_2d
from servos import MoveServo, MoveSyncTime, MoveSyncTime, ReturnToNeutral, MoveToPoint, steps
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

def half_step_init(phase):
    half_width = SWING_WIDTH / 2
    
    # Ruch sinusoidalny dla płynności
    t = phase  # [0,1]
    angle = math.pi * t
    
    x_l = -half_width * (1 - math.cos(angle))/2 + X_OFFSET
    z_l = BASE_Z + SWING_HEIGHT/2 * math.sin(angle)
    
    x_r = half_width * (1 - math.cos(angle))/2 + X_OFFSET
    z_r = BASE_Z
    
    return  x_l, z_l, x_r, z_r

def half_step_end(phase):
    half_width = SWING_WIDTH / 2
    t = phase  # [0,1]
    angle = math.pi * t

    # Lewa noga idzie od -half_width do środka
    x_l = -half_width * (1 + math.cos(angle))/2 + X_OFFSET
    z_l = BASE_Z

    # Prawa noga idzie od +half_width do środka
    x_r = half_width * (1 + math.cos(angle))/2 + X_OFFSET
    z_r = BASE_Z + SWING_HEIGHT/2 * math.sin(angle)

    return x_l, z_l, x_r, z_r

def runHalfStepInit(duration):
    start = time.perf_counter()
    end_time = start + duration

    while time.perf_counter() < end_time:
        now = time.perf_counter()
        dt = now - start
        phase = dt / duration  # normalizacja do [0,1]

        # Oblicz pozycje stóp z funkcji inicjalizującej
        x_l, z_l, x_r, z_r = half_step_init(phase)

        # ====== IK dla lewej nogi (przód) ======
        ik_l = solve_ik_2d(x_l, z_l, L1, L2, elbow_up=True)
        if ik_l is not None:
            t1_l, t2_l = ik_l
            deg1_l = math.degrees(t1_l)
            deg2_l = math.degrees(t2_l)
            deg3_l = deg1_l + deg2_l
            MoveServo(6, deg1_l)  # Lewe biodro
            MoveServo(7, deg3_l)  # Lewe kolano

        # ====== IK dla prawej nogi (tył) ======
        ik_r = solve_ik_2d(-x_r, z_r, L1, L2, elbow_up=False)
        if ik_r is not None:
            t1_r, t2_r = ik_r
            deg1_r = math.degrees(t1_r)
            deg2_r = math.degrees(t2_r)
            deg3_r = deg1_r + deg2_r
            MoveServo(2, deg1_r)  # Prawe biodro
            MoveServo(3, deg3_r)  # Prawe kolano

        # ====== Opóźnienie ======
        time.sleep(0.02)
    
    print(f"Zakończono inicjalizację pozycji startowej w czasie {duration}s")

def runHalfStepEnd(duration):
    start = time.perf_counter()
    end_time = start + duration

    while time.perf_counter() < end_time:
        now = time.perf_counter()
        dt = now - start
        phase = dt / duration  # normalizacja do [0,1]

        # Oblicz pozycje stóp z funkcji inicjalizującej
        x_l, z_l, x_r, z_r = half_step_end(phase)

        # ====== IK dla lewej nogi (przód) ======
        ik_l = solve_ik_2d(x_l, z_l, L1, L2, elbow_up=True)
        if ik_l is not None:
            t1_l, t2_l = ik_l
            deg1_l = math.degrees(t1_l)
            deg2_l = math.degrees(t2_l)
            deg3_l = deg1_l + deg2_l
            MoveServo(6, deg1_l)  # Lewe biodro
            MoveServo(7, deg3_l)  # Lewe kolano

        # ====== IK dla prawej nogi (tył) ======
        ik_r = solve_ik_2d(-x_r, z_r, L1, L2, elbow_up=False)
        if ik_r is not None:
            t1_r, t2_r = ik_r
            deg1_r = math.degrees(t1_r)
            deg2_r = math.degrees(t2_r)
            deg3_r = deg1_r + deg2_r
            MoveServo(2, deg1_r)  # Prawe biodro
            MoveServo(3, deg3_r)  # Prawe kolano

        # ====== Opóźnienie ======
        time.sleep(0.02)
    
    print(f"Zakończono inicjalizację pozycji końcowej w czasie {duration}s")

def tilt(phase):

    # --- FAZA 1: stałe przechylenie w prawo ---
    if phase < HOLD_TIME:
        left_foot_tilt = 90 - LEFT_FOOT_TILT
        right_foot_tilt = 90 - RIGHT_FOOT_TILT
        left_hip_tilt = 90
        right_hip_tilt = 90 - RIGHT_HIP_TILT
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    # --- FAZA 2: przejazd prawo -> lewo ---
    elif phase < 0.5:
        progress = (phase - HOLD_TIME) / (0.5 - HOLD_TIME)
        left_foot_tilt = 90 - LEFT_FOOT_TILT + (2 * LEFT_FOOT_TILT * progress)
        right_foot_tilt = 90 - RIGHT_FOOT_TILT + (2 * RIGHT_FOOT_TILT * progress)
        left_hip_tilt = 90 + (LEFT_HIP_TILT * progress)
        right_hip_tilt = 90 - RIGHT_HIP_TILT + (RIGHT_HIP_TILT * progress)
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    # --- FAZA 3: stałe przechylenie w lewo ---
    elif phase < 0.5 + HOLD_TIME:
        left_foot_tilt = 90 + LEFT_FOOT_TILT
        right_foot_tilt = 90 + RIGHT_FOOT_TILT
        left_hip_tilt = 90 + LEFT_HIP_TILT
        right_hip_tilt = 90
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt

    # --- FAZA 4: przejazd lewo -> prawo ---
    else:
        progress = (phase - (0.5 + HOLD_TIME)) / (1.0 - (0.5 + HOLD_TIME))
        left_foot_tilt = 90 + LEFT_FOOT_TILT - (2 * LEFT_FOOT_TILT * progress)
        right_foot_tilt = 90 + RIGHT_FOOT_TILT - (2 * RIGHT_FOOT_TILT * progress)
        left_hip_tilt = 90 + LEFT_HIP_TILT - (LEFT_HIP_TILT * progress)
        right_hip_tilt = 90 - (RIGHT_HIP_TILT * progress)
        return left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt
            
def runTrotGaitTwoLegs(num_cycles=2):
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


MoveToPoint(-15, -90, "right", 2000, 10)
MoveToPoint(-15, -90, "left", 2000, 10)


# MoveSyncTime({
#     8: 90 + RIGHT_FOOT_TILT,   
#     4: 90 + LEFT_FOOT_TILT,
#     5: 90, 
#     1: 90 + LEFT_HIP_TILT   
# }, 0.8, 10 )

# runHalfStepInit(CYCLE_TIME * SWING_TIME)

# MoveSyncTime({
#     8: 90 - RIGHT_FOOT_TILT,   
#     4: 90 - LEFT_FOOT_TILT,
#     5: 90 - RIGHT_HIP_TILT, 
#     1: 90    
# }, 0.8, 10 )

runTrotGaitTwoLegs(7)

# runHalfStepEnd(CYCLE_TIME * SWING_TIME)
# time.sleep(0.3)

ReturnToNeutral()


import math
import time

from ik import solve_ik_2d
from servos import MoveServo
from config import HOLD_TIME, L1, L2, CYCLE_TIME, FOOT_TILT, HIP_TILT, SWING_WIDTH, SWING_HEIGHT, SWING_TIME, X_OFFSET, BASE_Z

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

def initial_half_step(phase, do_swing=True):
    half_width = SWING_WIDTH / 2
    
    # Prosty, liniowy ruch przez cały czas (phase 0.0 → 1.0)
    if do_swing:
        angle = math.pi * phase
        x = -half_width * phase + X_OFFSET
        z = BASE_Z + SWING_HEIGHT * math.sin(angle)
    else:
        x = half_width * phase + X_OFFSET
        z = BASE_Z
    
    return x, z

def final_half_step(phase, do_swing=True):
    half_width = SWING_WIDTH / 2
    
    if do_swing:
        # Swing z tyłu do środka
        angle = math.pi * phase  # 0 → π
        x = half_width - half_width * phase + X_OFFSET  # tył → środek
        z = BASE_Z + SWING_HEIGHT * math.sin(angle)
    else:
        # Stance z przodu do środka
        x = -half_width + half_width * phase + X_OFFSET  # przód → środek
        z = BASE_Z
    
    return x, z

def apply_ik(x, z, servo_hip, servo_knee, elbow_up):
    ik = solve_ik_2d(x, z, L1, L2, elbow_up=elbow_up)
    if ik:
        t1, t2 = ik
        MoveServo(servo_hip, math.degrees(t1))
        MoveServo(servo_knee, math.degrees(t1) + math.degrees(t2))

def apply_tilt(phase):
    left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt = tilt(phase)
    MoveServo(4, left_foot_tilt)
    MoveServo(8, right_foot_tilt)
    MoveServo(1, left_hip_tilt)
    MoveServo(5, right_hip_tilt)

def tilt(phase):
    phase = phase % 1.0
    p1, p2, p3 = HOLD_TIME, 0.5, 0.5 + HOLD_TIME
    
    if phase < p1:
        progress = -1.0
    elif phase < p2:
        progress = -1.0 + 2.0 * (phase - p1) / (p2 - p1)
    elif phase < p3:
        progress = 1.0
    else:
        progress = 1.0 - 2.0 * (phase - p3) / (1.0 - p3)
    
    hip_progress = (progress + 1.0) / 2.0
    return (
        90 + FOOT_TILT * progress,
        90 + FOOT_TILT * progress,
        90 + HIP_TILT * hip_progress,
        90 - HIP_TILT * (1.0 - hip_progress)
    )

def runInitialHalfStep():
    start = time.perf_counter()
    half_step_time = CYCLE_TIME/2
    transition_time = (CYCLE_TIME/2) * 0.35  # czas na zmianę przechyłu po wylądowaniu
    
    print("initial half step:")
    while time.perf_counter() - start < half_step_time + transition_time:
        elapsed = time.perf_counter() - start
        
        if elapsed < half_step_time:
            # Faza 1: Wykonywanie half-stepa
            phase = elapsed / half_step_time
            x_l, z_l = initial_half_step(phase, do_swing=False)
            x_r, z_r = initial_half_step(phase, do_swing=True)
            
            apply_ik(-x_l, z_l, 2, 3, elbow_up=False)
            apply_ik(x_r, z_r, 6, 7, elbow_up=True)
            
            # Statyczny przechył w prawo przez cały half-step
            progress = 1.0
            hip_progress = 1.0
            MoveServo(4, 90 + FOOT_TILT * progress)
            MoveServo(8, 90 + FOOT_TILT * progress)
            MoveServo(1, 90 + HIP_TILT * hip_progress)
            MoveServo(5, 90 - HIP_TILT * (1.0 - hip_progress))
        else:
            # Faza 2: Zmiana przechyłu po wylądowaniu
            transition_phase = (elapsed - half_step_time) / transition_time
            progress = 1.0 - 2.0 * transition_phase  # +1.0 → -1.0
            hip_progress = 1.0 - transition_phase     # 1.0 → 0.0
            
            MoveServo(4, 90 + FOOT_TILT * progress)
            MoveServo(8, 90 + FOOT_TILT * progress)
            MoveServo(1, 90 + HIP_TILT * hip_progress)
            MoveServo(5, 90 - HIP_TILT * (1.0 - hip_progress))
        
        time.sleep(0.02)
    
    print("Initial half-step completed - ready for trot gait")

def runFinalHalfStep():
    start = time.perf_counter()
    half_step_time = CYCLE_TIME/2
    transition_time = (CYCLE_TIME/2) * 0.35
    
    print("final half step:")
    while time.perf_counter() - start < half_step_time + transition_time:
        elapsed = time.perf_counter() - start

        if elapsed < half_step_time:
            # Faza 1: Wykonywanie final half-stepa
            phase = elapsed / half_step_time
            
            x_l, z_l = final_half_step(phase, do_swing=True)   # prawa swing
            x_r, z_r = final_half_step(phase, do_swing=False)  # lewa stance
            
            apply_ik(-x_l, z_l, 2, 3, elbow_up=False)
            apply_ik(x_r, z_r, 6, 7, elbow_up=True)
            
            # Statyczny przechył w lewo przez cały half-step
            progress = -1.0
            hip_progress = 0.0
            MoveServo(4, 90 + FOOT_TILT * progress)
            MoveServo(8, 90 + FOOT_TILT * progress)
            MoveServo(1, 90 + HIP_TILT * hip_progress)
            MoveServo(5, 90 - HIP_TILT * (1.0 - hip_progress))
        else:
            # Faza 2: Zmiana przechyłu do neutralnego PO wylądowaniu
            transition_phase = (elapsed - half_step_time) / transition_time
            progress = -1.0 + transition_phase  # -1.0 → 0.0
            hip_progress = transition_phase * 0.5  # 0.0 → 0.5 (centrum)
            
            MoveServo(4, 90 + FOOT_TILT * progress)
            MoveServo(8, 90 + FOOT_TILT * progress)
            MoveServo(1, 90 + 0 * hip_progress)
            MoveServo(5, 90 - 0 * (1.0 - hip_progress))
        
        time.sleep(0.02)
    
    print("Final half-step completed - robot stopped in neutral position")

def runTrotGaitTwoLegs(num_cycles):
    print(f"Rozpoczynanie chodu trot gait na {num_cycles} cykli...")
    start = time.perf_counter()
    end_time = start + num_cycles * CYCLE_TIME

    while time.perf_counter() < end_time:
        phase = ((time.perf_counter() - start) % CYCLE_TIME) / CYCLE_TIME
        phase_left = (phase + 0.5) % 1.0

        x_l, z_l = trot_gait(phase)
        x_r, z_r = trot_gait(phase_left)

        apply_ik(-x_l, z_l, 2, 3, elbow_up=False)
        apply_ik(x_r, z_r, 6, 7, elbow_up=True)
        
        apply_tilt(phase)

        time.sleep(0.02)
    
    print(f"Zakończono {num_cycles} cykle chodu")


runInitialHalfStep()
runTrotGaitTwoLegs(5)
runFinalHalfStep()



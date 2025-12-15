import time
import numpy as np
import math
from servos import MoveServo

# ========================================
# STAŁE KONFIGURACYJNE
# ========================================
TOTAL_HEIGHT = 0.302
SWING_WIDTH = 0.05
SWING_HEIGHT = 0.03
SWING_TIME = 0.35
Z_OFFSET = 0.02
X_OFFSET = 0.0

LEFT_FOOT_TILT = 15
RIGHT_FOOT_TILT = 15
LEFT_HIP_TILT = 5
RIGHT_HIP_TILT = 5
HOLD_TIME = SWING_TIME * 0.75

# ========================================
# FUNKCJE CHODU
# ========================================

def trot_gait(phase: float, swing_width: float, swing_height: float):
    """Generuje trajektorię chodu dla jednej nogi"""
    half_w = swing_width / 2

    if phase < SWING_TIME:
        t = phase / SWING_TIME
        angle = math.pi * t
        x = -half_w + swing_width * t + X_OFFSET
        z = (TOTAL_HEIGHT - Z_OFFSET) - swing_height * math.sin(angle)  
    else:
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = half_w - swing_width * t + X_OFFSET
        z = TOTAL_HEIGHT - Z_OFFSET
    return x, z

def tilt(phase):
    """Oblicza przechyły dla stabilizacji chodu"""
    phase = phase % 1.0

    p1 = HOLD_TIME
    p2 = 0.5
    p3 = 0.5 + HOLD_TIME

    if phase < p1:
        return (
            0 - LEFT_FOOT_TILT,
            0 - RIGHT_FOOT_TILT,
            0,
            0 - RIGHT_HIP_TILT
        )
    elif phase < p2:
        progress = (phase - p1) / (p2 - p1)
        return (
            0 - LEFT_FOOT_TILT + 2 * LEFT_FOOT_TILT * progress,
            0 - RIGHT_FOOT_TILT + 2 * RIGHT_FOOT_TILT * progress,
            0 + LEFT_HIP_TILT * progress,
            0 - RIGHT_HIP_TILT + RIGHT_HIP_TILT * progress
        )
    elif phase < p3:
        return (
            0 + LEFT_FOOT_TILT,
            0 + RIGHT_FOOT_TILT,
            0 + LEFT_HIP_TILT,
            0
        )
    else:
        progress = (phase - p3) / (1.0 - p3)
        return (
            0 + LEFT_FOOT_TILT  - 2 * LEFT_FOOT_TILT * progress,
            0 + RIGHT_FOOT_TILT - 2 * RIGHT_FOOT_TILT * progress,
            0 + LEFT_HIP_TILT   - LEFT_HIP_TILT * progress,
            0 - RIGHT_HIP_TILT * progress
        )

def solve_ik_3d(x, y, zt, leg):
    """Rozwiązuje kinematykę odwrotną dla pozycji nogi"""
    z = zt - 0.128
    l1, l2, l3 = 0.04, 0.067, 0.067
    hip_roll = np.arctan2(y, z)
    
    D = np.sqrt(y**2 + z**2)
    r = np.sqrt(x**2 + (D-l1)**2)
    
    cos_knee = (l2**2 + l3**2 - r**2) / (2 * l2 * l3)
    
    if cos_knee < -1 or cos_knee > 1:
        raise ValueError(f"Pozycja ({x:.3f}, {y:.3f}, {z:.3f}) poza zasięgiem")
    
    knee_pitch = np.pi - np.arccos(cos_knee)
    alpha = np.arctan2(x, (D-l1))
    cos_beta = (l2**2 + r**2 - l3**2) / (2 * l2 * r)
    beta = np.arccos(np.clip(cos_beta, -1, 1))
    hip_pitch = -(alpha + beta)
    
    joint_targets = [0.0] * 8
    if leg == "left":
        joint_targets[0] = hip_roll 
        joint_targets[1] = hip_pitch
        joint_targets[2] = knee_pitch + hip_pitch
        joint_targets[3] = -hip_roll
    else:  # right
        joint_targets[4] = hip_roll 
        joint_targets[5] = hip_pitch
        joint_targets[6] = -(knee_pitch + hip_pitch)
        joint_targets[7] = -hip_roll

    return joint_targets

def calculate_trot_gait(phase, swing_width, swing_height):
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    left_y = 0
    
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    right_y = 0
    
    return (left_x, left_y, left_z), (right_x, right_y, right_z)

def calculate_tilt(phase, tilt_gain):
    left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt = tilt(phase)

    left_foot_tilt  *= tilt_gain
    right_foot_tilt *= tilt_gain
    left_hip_tilt   *= tilt_gain
    right_hip_tilt  *= tilt_gain

    return left_hip_tilt, left_foot_tilt, right_hip_tilt, right_foot_tilt

def combine_leg_targets(left_pos, right_pos, left_hip_tilt, left_foot_tilt, right_hip_tilt, right_foot_tilt):
    left_x, left_y, left_z = left_pos
    right_x, right_y, right_z = right_pos

    left_targets = solve_ik_3d(left_x,  left_y, left_z,  "left")
    right_targets = solve_ik_3d(right_x, right_y, right_z, "right")

    left_targets[0] = -np.radians(left_hip_tilt)
    left_targets[3] =  np.radians(left_foot_tilt)

    right_targets[4] = -np.radians(right_hip_tilt)
    right_targets[7] = np.radians(right_foot_tilt)

    joint_targets = [0.0] * 8
    for i in range(8):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]

    return joint_targets

def update_robot(joint_targets):
    # Lewa noga
    hip_roll_L   = 90 - math.degrees(joint_targets[0])
    hip_pitch_L  = 90 + math.degrees(joint_targets[1])
    knee_L       = 90 + math.degrees(joint_targets[2])
    ankle_L      = 90 + math.degrees(joint_targets[3])

    # Prawa noga
    hip_roll_R   = 90 - math.degrees(joint_targets[4])
    hip_pitch_R  = 90 - math.degrees(joint_targets[5])
    knee_R       = 90 + math.degrees(joint_targets[6])
    ankle_R      = 90 + math.degrees(joint_targets[7])

    # Wysyłanie do serw
    MoveServo(1, hip_roll_L)
    MoveServo(2, hip_pitch_L)
    MoveServo(3, knee_L)
    MoveServo(4, ankle_L)

    MoveServo(5, hip_roll_R)
    MoveServo(6, hip_pitch_R)
    MoveServo(7, knee_R)
    MoveServo(8, ankle_R)

# ========================================
# GŁÓWNA PĘTLA CHODU
# ========================================

def walk(speed=2.5, swing_width=0.05, swing_height=0.0, tilt_gain=0.0, duration=None):
    phase = 0.0
    dt = 1./240.
    start_time = time.time()

    while True:
        if duration and (time.time() - start_time) > duration:
            break

        phase += speed * dt
        phase = phase % 1.0

        left_pos, right_pos = calculate_trot_gait(phase, swing_width, swing_height)
        left_hip_tilt, left_foot_tilt, right_hip_tilt, right_foot_tilt = calculate_tilt(phase, tilt_gain)
        joint_targets = combine_leg_targets(left_pos, right_pos, left_hip_tilt, left_foot_tilt, right_hip_tilt, right_foot_tilt)
        update_robot(joint_targets)

        time.sleep(dt)

# ========================================
# PRZYKŁAD UŻYCIA
# ========================================

if __name__ == "__main__":
    # Podstawowy chód
    walk(speed=1.5, swing_width=0.02, swing_height=0.02, tilt_gain=0.9)

import math
import numpy as np

def solve_ik_2d(x_target, z_target, l1, l2, elbow_up=False):
    # d = math.hypot(x_target, z_target)
    # if d > l1 + l2 or d < abs(l1 - l2):
    #     return None

    cos_theta2 = (x_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(z_target, x_target) - math.atan2(k2, k1)

    alpha = theta1
    betha = theta1 + theta2
    # print(math.degrees(alpha), math.degrees(betha))

    return -theta1, -theta2

def solve_ik_3d(x, y, zt, leg):
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

    hip_roll_L   = 90 - math.degrees(joint_targets[0])
    hip_pitch_L  = 90 + math.degrees(joint_targets[1])
    knee_L       = 90 + math.degrees(joint_targets[2])
    ankle_L      = 90 + math.degrees(joint_targets[3])

    # Prawa noga
    hip_roll_R   = 90 - math.degrees(joint_targets[4])
    hip_pitch_R  = 90 - math.degrees(joint_targets[5])
    knee_R       = 90 + math.degrees(joint_targets[6])
    ankle_R      = 90 + math.degrees(joint_targets[7])

    return joint_targets

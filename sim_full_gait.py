import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# ========================================
# STAŁE KONFIGURACYJNE
# ========================================
SWING_WIDTH = 0.04
SWING_HEIGHT = 0.03
SWING_TIME = 0.5
Z_OFFSET = 0.011
X_OFFSET = 0.0
LEFT_Y = 0.04
RIGHT_Y = -0.04

# ========================================
# INICJALIZACJA
# ========================================
def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.290], useFixedBase=True)
    return robot

def create_debug_sliders():
    """Tworzy slidery do sterowania parametrami"""
    sliders = {
        'camera_distance': p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5),
        'camera_yaw': p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 56),
        'camera_pitch': p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0),
        'camera_height': p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0),
        'speed': p.addUserDebugParameter("  Predkosc chodu", 0.1, 5.0, 1.0),
        'swing_width': p.addUserDebugParameter("  Szerokosc kroku", 0.05, 0.4, SWING_WIDTH),
        'swing_height': p.addUserDebugParameter("  Wysokosc kroku", 0.01, 0.15, SWING_HEIGHT)
    }
    return sliders

def trot_gait(phase: float, swing_width: float, swing_height: float):
    """Zwraca pozycję (x, z) stopy dla danej fazy chodu (0-1)"""
    half_w = swing_width / 2

    if phase < SWING_TIME:
        # Faza swing - noga w powietrzu
        t = phase / SWING_TIME
        angle = math.pi * (1 - t)
        x = half_w * math.cos(angle) + X_OFFSET
        z = Z_OFFSET + swing_height * math.sin(angle)
    else:
        # Faza stance - noga na ziemi
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = half_w - swing_width * t + X_OFFSET
        z = Z_OFFSET

    return x, z

def solve_ik(x_target, z_target, leg, elbow_up=False):
    """Rozwiązuje kinematykę odwrotną dla danej nogi"""
    x = x_target
    z = z_target - 0.090 - 0.077 + 0.033
    l1 = 0.067
    l2 = 0.067

    cos_theta2 = (x*x + z*z - l1*l1 - l2*l2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(z, x) - math.atan2(k2, k1)

    joint_targets = [0.0] * 14
    if leg == "left":
        joint_targets[0] = 0
        joint_targets[1] = -theta1 - 1.57
        joint_targets[2] = -theta1 - 1.57
        joint_targets[3] = -theta2 - theta1 - 1.57
        joint_targets[4] = theta2 + theta1 + 1.57
        joint_targets[5] = 0
        joint_targets[6] = 0
    else:
        joint_targets[7] = 0
        joint_targets[8] = -theta1 - 1.57
        joint_targets[9] = theta1 + 1.57
        joint_targets[10] = theta2 + theta1 + 1.57
        joint_targets[11] = -theta2 - theta1 - 1.57
        joint_targets[12] = 0
        joint_targets[13] = 0
    
    return joint_targets

def calculate_leg_positions(phase, swing_width, swing_height):
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    
    return (left_x, left_z), (right_x, right_z)

def combine_leg_targets(left_pos, right_pos):
    left_x, left_z = left_pos
    right_x, right_z = right_pos
    
    left_targets = solve_ik(left_x, left_z, "left")
    right_targets = solve_ik(right_x, right_z, "right")
    
    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    
    return joint_targets

def apply_joint_targets(robot, joint_targets):
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        p.resetJointState(robot, i, joint_targets[i])

def update_camera(robot, sliders):
    """Aktualizuje pozycję kamery na podstawie sliderów"""
    camera_distance = p.readUserDebugParameter(sliders['camera_distance'])
    camera_yaw = p.readUserDebugParameter(sliders['camera_yaw'])
    camera_pitch = p.readUserDebugParameter(sliders['camera_pitch'])
    camera_height = p.readUserDebugParameter(sliders['camera_height'])
    
    robot_pos, _ = p.getBasePositionAndOrientation(robot)
    camera_target = [robot_pos[0], robot_pos[1], robot_pos[2] + camera_height]
    
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target
    )

def read_gait_parameters(sliders):
    """Odczytuje parametry chodu ze sliderów"""
    return {
        'speed': p.readUserDebugParameter(sliders['speed']),
        'swing_width': p.readUserDebugParameter(sliders['swing_width']),
        'swing_height': p.readUserDebugParameter(sliders['swing_height'])
    }

def main():
    robot = init_simulation()
    sliders = create_debug_sliders()
    
    phase = 0.0
    dt = 1./240.
    
    while True:
        params = read_gait_parameters(sliders)
        
        phase += params['speed'] * dt
        phase = phase % 1.0
        
        left_pos, right_pos = calculate_leg_positions(
            phase, params['swing_width'], params['swing_height']
        )
        joint_targets = combine_leg_targets(left_pos, right_pos)
        
        apply_joint_targets(robot, joint_targets)

        update_camera(robot, sliders)
        
        p.stepSimulation()
        time.sleep(dt)

if __name__ == "__main__":
    main()
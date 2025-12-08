import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# ========================================
# STAŁE KONFIGURACYJNE
# ========================================
TOTAL_HEIGHT = 0.302
SWING_WIDTH = 0.05
SWING_HEIGHT = 0.03
SWING_TIME = 0.5
Z_OFFSET = 0.02            # minimalne ugiecie nóg aby mieć zasięg w poziomie         
X_OFFSET = 0.0
GAIT_SPEED = 0.8            # cykle/s

# ========================================
# INICJALIZACJA
# ========================================
def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=True)
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
        angle = math.pi * t
        x = -half_w + swing_width * t + X_OFFSET
        z = (TOTAL_HEIGHT - Z_OFFSET) - swing_height * math.sin(angle)  # ZMIANA: minus zamiast plus
    else:
        # Faza stance - noga na ziemi
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = half_w - swing_width * t + X_OFFSET
        z = TOTAL_HEIGHT - Z_OFFSET

    return x, z

def solve_ik_3d(x, y, zt, leg, elbow_up=False):
    z = zt - 0.128
    l1, l2, l3 = 0.04, 0.067, 0.067
    hip_roll = np.arctan2(y, z)
    
    D = np.sqrt(y**2 + z**2)
    print(f"D: {D:.3f}")
    
    r = np.sqrt(x**2 + (D-l1)**2)
    print(f"r: {r:.3f}")
    
    cos_knee = (l2**2 + l3**2 - r**2) / (2 * l2 * l3)
    print(f"cos_knee: {cos_knee:.3f}")
    
    if cos_knee < -1 or cos_knee > 1:
        raise ValueError(f"Pozycja ({x:.3f}, {y:.3f}, {z:.3f}) jest poza zasięgiem nogi")
    
    knee_pitch = np.pi - np.arccos(cos_knee)

    alpha = np.arctan2(x, (D-l1))
    cos_beta = (l2**2 + r**2 - l3**2) / (2 * l2 * r)
    beta = np.arccos(np.clip(cos_beta, -1, 1))
    hip_pitch = -(alpha + beta)

    print(f"Noga {leg} - Obliczone kąty stawów:")
    print(f"  Hip Roll:  {np.degrees(hip_roll):7.2f}°")
    print(f"  Hip Pitch: {np.degrees(hip_pitch):7.2f}°")
    print(f"  Knee Pitch: {np.degrees(knee_pitch):7.2f}°")
    
    # Mapowanie na joiny robota
    joint_targets = [0.0] * 14
    if leg == "left":
        joint_targets[0] = hip_roll 
        joint_targets[1] = hip_pitch
        joint_targets[2] = hip_pitch
        joint_targets[3] = knee_pitch + hip_pitch
        joint_targets[4] = -(knee_pitch + hip_pitch)
        joint_targets[5] = -hip_roll
        joint_targets[6] = 0  # fixed joint
    else:  # right
        joint_targets[7] = hip_roll 
        joint_targets[8] = hip_pitch
        joint_targets[9] = -hip_pitch
        joint_targets[10] = -(knee_pitch + hip_pitch)
        joint_targets[11] = knee_pitch + hip_pitch
        joint_targets[12] = -hip_roll
        joint_targets[13] = 0  # fixed joint

    return joint_targets

def calculate_leg_positions(phase, swing_width, swing_height):
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    left_y = 0
    
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    right_y = 0
    
    return (left_x, left_y, left_z), (right_x, right_y, right_z)

def combine_leg_targets(left_pos, right_pos):
    left_x, left_y, left_z = left_pos
    right_x, right_y, right_z = right_pos
    
    left_targets = solve_ik_3d(left_x, left_y, left_z, "left")
    right_targets = solve_ik_3d(right_x, right_y, right_z, "right")
    
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
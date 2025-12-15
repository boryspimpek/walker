import math
import pybullet as p
import pybullet_data
import time
import numpy as np

TOTAL_HEIGHT = 0.302
SWING_TIME = 0.4
Z_OFFSET = 0.02
X_OFFSET = 0.0

CYCLE_PERIOD = 4.0  # Okres pełnego cyklu chodu w sekundach

def initialize_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
    
    return robot

def init_pose(robot):
    left_targets = solve_ik_3d(0.0, 0.05, TOTAL_HEIGHT - Z_OFFSET, "left", robot)
    right_targets = solve_ik_3d(0.0, -0.05, TOTAL_HEIGHT - Z_OFFSET, "right", robot)

    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        p.resetJointState(robot, i, joint_targets[i])

def create_ui_sliders():
    sliders = {
        'camera_distance': p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5),
        'camera_yaw': p.addUserDebugParameter("  Obrot", -180, 180, 90),
        'camera_pitch': p.addUserDebugParameter("  Pitch", -89, 89, 0),
        'camera_height': p.addUserDebugParameter("  Wysokosc", -1.0, 1.0, 0.0),
        'swing_height': p.addUserDebugParameter("  Wysokosc kroku", 0.0, 0.2, 0.0),
        'swing_width': p.addUserDebugParameter("  Szerokosc kroku", 0.0, 0.2, 0.0),
        'max_tilt_angle': p.addUserDebugParameter("  Maksymalny kat wychylenia", 0.0, 30.0, 15.0),
    }
    return sliders

def solve_ik_3d(x, y, zt, leg, robot, phase, max_tilt_angle, elbow_up=False):
    z = zt - 0.128
    l1, l2, l3 = 0.04, 0.067, 0.067
    hip_roll = np.arctan2(y, z)
    
    D = np.sqrt(y**2 + z**2)
    r = np.sqrt(x**2 + (D-l1)**2)
    
    cos_knee = (l2**2 + l3**2 - r**2) / (2 * l2 * l3)
    
    if cos_knee < -1 or cos_knee > 1:
        raise ValueError(f"Pozycja ({x:.3f}, {y:.3f}, {z:.3f}) jest poza zasięgiem nogi")
    
    knee_pitch = np.pi - np.arccos(cos_knee)

    alpha = np.arctan2(x, (D-l1))
    cos_beta = (l2**2 + r**2 - l3**2) / (2 * l2 * r)
    beta = np.arccos(np.clip(cos_beta, -1, 1))
    hip_pitch = -(alpha + beta)

    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    base_euler = p.getEulerFromQuaternion(base_orn)

    # Pobierz wartości tilt
    left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt = tilt(phase, max_tilt_angle)
    
    # Konwersja stopni na radiany
    right_foot_tilt_rad = np.radians(right_foot_tilt)
    left_foot_tilt_rad = np.radians(left_foot_tilt)
    right_hip_tilt_rad = np.radians(right_hip_tilt)
    left_hip_tilt_rad = np.radians(left_hip_tilt)

    # Mapowanie na joiny robota z zastosowaniem tilt
    joint_targets = [0.0] * 14
    if leg == "left":
        joint_targets[0] = -left_hip_tilt_rad  # left hip roll
        joint_targets[1] = hip_pitch
        joint_targets[2] = hip_pitch
        joint_targets[3] = knee_pitch + hip_pitch
        joint_targets[4] = -(knee_pitch + hip_pitch)
        joint_targets[5] = left_foot_tilt_rad  # left foot roll
        joint_targets[6] = 0  # fixed joint
    else:  # right
        joint_targets[7] = -right_hip_tilt_rad  # right hip roll
        joint_targets[8] = hip_pitch
        joint_targets[9] = -hip_pitch
        joint_targets[10] = -(knee_pitch + hip_pitch)
        joint_targets[11] = knee_pitch + hip_pitch
        joint_targets[12] = right_foot_tilt_rad  # right foot roll
        joint_targets[13] = 0  # fixed joint

    return joint_targets

def trot_gait(phase, swing_width, swing_height):
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

def tilt(phase, max_tilt_angle):
    a = 0.05
    b = 0.2451325
    d = 0.080

    gamma_max = np.radians(90 + max_tilt_angle)
    c_max = np.sqrt(a**2 + b**2 - 2 * a * b * np.cos(gamma_max))
        
    cos_beta_max = (a**2 + c_max**2 - b**2) / (2 * a * c_max)
    beta_max = np.arccos(np.clip(cos_beta_max, -1, 1))
    
    cos_beta_prim_max = (b**2 + c_max**2 - d**2) / (2 * b * c_max)
    beta_prim_max = np.arccos(np.clip(cos_beta_prim_max, -1, 1))
    
    min_tilt_angle = np.degrees(beta_max + beta_prim_max) - 90

    center = (max_tilt_angle + min_tilt_angle) / 2
    amplitude = (max_tilt_angle - min_tilt_angle) / 2
    mid_swing = SWING_TIME / 2

    phase = phase % 1.0
    
    tilt_angle = center + amplitude * np.cos(2 * np.pi * (phase- mid_swing))
    
    gamma = np.radians(90 + tilt_angle)
    c = np.sqrt(a**2 + b**2 - 2 * a * b * np.cos(gamma))
    
    cos_alfa = (b**2 + c**2 - a**2) / (2 * b * c)
    alfa = np.arccos(np.clip(cos_alfa, -1, 1))
    
    cos_beta = (a**2 + c**2 - b**2) / (2 * a * c)
    beta = np.arccos(np.clip(cos_beta, -1, 1))
    
    cos_delta = (b**2 + d**2 - c**2) / (2 * b * d)
    delta = np.arccos(np.clip(cos_delta, -1, 1))
    
    cos_beta_prim = (b**2 + c**2 - d**2) / (2 * b * c)
    beta_prim = np.arccos(np.clip(cos_beta_prim, -1, 1))
    
    cos_alfa_prim = (c**2 + d**2 - b**2) / (2 * c * d)
    alfa_prim = np.arccos(np.clip(cos_alfa_prim, -1, 1))
    
    return [
        np.degrees(beta + beta_prim) - 90,
        -tilt_angle,
        -(np.degrees(delta) - 90),
        np.degrees(alfa + alfa_prim) - 90
    ]
    
def get_trot_leg_positions(time_sec, swing_width, swing_height):
    # Normalizacja czasu do zakresu [0, 1]
    phase = (time_sec % CYCLE_PERIOD) / CYCLE_PERIOD
        
    # Lewa noga
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    left_y = 0  
    
    # Prawa noga - przesunięta o pół cyklu
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    right_y = 0
    
    return (left_x, left_y, left_z), (right_x, right_y, right_z)

def combine_leg_targets(left_targets, right_targets):
    """Łączy cele dla lewej i prawej nogi w jeden wektor."""
    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    return joint_targets

def apply_joint_targets(robot, joint_targets, force=500):
    """Aplikuje docelowe pozycje do przegubów robota."""
    for jid in range(14):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets[jid],
            force=force
        )

def update_camera(robot, sliders):
    """Aktualizuje pozycję kamery na podstawie sliderów."""
    cam_dist = p.readUserDebugParameter(sliders['camera_distance'])
    cam_yaw = p.readUserDebugParameter(sliders['camera_yaw'])
    cam_pitch = p.readUserDebugParameter(sliders['camera_pitch'])
    cam_height = p.readUserDebugParameter(sliders['camera_height'])

    base_pos, _ = p.getBasePositionAndOrientation(robot)
    cam_target = [base_pos[0], base_pos[1], base_pos[2] + cam_height]

    p.resetDebugVisualizerCamera(
        cameraDistance=cam_dist,
        cameraYaw=cam_yaw,
        cameraPitch=cam_pitch,
        cameraTargetPosition=cam_target
    )

def debug_info(robot):
    """Wyświetla informacje debugowania."""
    print("\n" + "=" * 50)
    print("AKTUALNE KĄTY PRZEGUBÓW:")

    for i in range(min(20, p.getNumJoints(robot))):
        angle = p.getJointState(robot, i)[0]
        print(f"Joint {i}: {angle:7.2f}°")

    print("\n" + "=" * 50)
    print("base_link orientation:")
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    base_euler = p.getEulerFromQuaternion(base_orn)
    print(f"Orinetation (degrees): ({np.degrees(base_euler[0]):.2f}, {np.degrees(base_euler[1]):.2f}, {np.degrees(base_euler[2]):.2f})")

    print("=" * 50 + "\n")
    print("Left Foot orientation:")
    link_state = p.getLinkState(robot, 6)
    link_pos = link_state[0]
    link_orn = link_state[1]  
    link_euler = p.getEulerFromQuaternion(link_orn)
    print(f"Position: ({link_pos[0]:.3f}, {link_pos[1]:.3f}, {link_pos[2]:.3f})")
    print(f"Orientation (degrees): ({np.degrees(link_euler[0]):.2f}, {np.degrees(link_euler[1]):.2f}, {np.degrees(link_euler[2]):.2f})")
    print("=" * 50 + "\n")

def main():
    """Główna pętla symulacji."""
    robot = initialize_simulation()
    sliders = create_ui_sliders()

    p.setGravity(0, 0, 0)
    # init_pose(robot)
        
    for _ in range(int(2 * 240)):  # 240 Hz
        update_camera(robot, sliders)
        debug_info(robot)
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.setGravity(0, 0, -9.81)

    start_time = time.time()
    
    while True:
        current_time = time.time() - start_time
        
        # Odczytaj wartości ze sliderów
        swing_width = p.readUserDebugParameter(sliders['swing_width'])
        swing_height = p.readUserDebugParameter(sliders['swing_height'])
        max_tilt_angle = p.readUserDebugParameter(sliders['max_tilt_angle'])
        
        # Oblicz fazę dla lewej nogi
        phase = (current_time % CYCLE_PERIOD) / CYCLE_PERIOD
        

        (left_x, left_y, left_z), (right_x, right_y, right_z) = get_trot_leg_positions(
            current_time, swing_width, swing_height)

        try:
            left_targets = solve_ik_3d(left_x, left_y, left_z, "left", robot, phase, max_tilt_angle)
            right_targets = solve_ik_3d(right_x, right_y, right_z, "right", robot, phase, max_tilt_angle)

            joint_targets = combine_leg_targets(left_targets, right_targets)
            apply_joint_targets(robot, joint_targets)
        except ValueError as e:
            print(f"IK Error: {e}")

        debug_info(robot)
        update_camera(robot, sliders)
        
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
import math
import pybullet as p
import pybullet_data
import time
import numpy as np
from servos import MoveServo

last_send_time = 0

TOTAL_HEIGHT = 0.302
SWING_TIME = 0.3
Z_OFFSET = 0.02
X_OFFSET = 0.0

CYCLE_PERIOD = 3.0  # Okres pełnego cyklu chodu w sekundach

def initialize_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.279], useFixedBase=False)
    
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
        'camera_pitch': p.addUserDebugParameter("  Pitch", -89, 89, -0),
        'camera_height': p.addUserDebugParameter("  Wysokosc", -1.0, 1.0, 0.0),
        'swing_height': p.addUserDebugParameter("  Wysokosc kroku", 0.0, 0.2, 0.0),
        'swing_width': p.addUserDebugParameter("  Szerokosc kroku", 0.0, 0.2, 0.0),
        'tilt_amplitude': p.addUserDebugParameter("  Tilt", 0.0, 0.1, 0),
    }
    return sliders

def solve_ik_3d(x, y, zt, leg, robot, elbow_up=False):
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

    # Mapowanie na joiny robota
    joint_targets = [0.0] * 14
    if leg == "left":
        joint_targets[0] = hip_roll 
        joint_targets[1] = hip_pitch
        joint_targets[2] = hip_pitch
        joint_targets[3] = knee_pitch + hip_pitch
        joint_targets[4] = -(knee_pitch + hip_pitch)
        joint_targets[5] = - hip_roll  
        joint_targets[6] = 0  # fixed joint
    else:  # right
        joint_targets[7] = -hip_roll 
        joint_targets[8] = hip_pitch
        joint_targets[9] = -hip_pitch
        joint_targets[10] = -(knee_pitch + hip_pitch)
        joint_targets[11] = knee_pitch + hip_pitch
        joint_targets[12] = hip_roll
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

def tilt(phase, tilt_amp):
    phase = phase % 1.0
    mid_swing = SWING_TIME / 2
    
    # Cosinus ma szczyt w 0, więc przesuwamy fazę
    return tilt_amp * math.cos(2 * math.pi * (phase - mid_swing))
    
def get_trot_leg_positions(time_sec, swing_width, swing_height, tilt_amp):
    # Normalizacja czasu do zakresu [0, 1]
    phase = (time_sec % CYCLE_PERIOD) / CYCLE_PERIOD
    
    # Lewa noga
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    left_y = tilt(left_phase, tilt_amp)  
    
    # Prawa noga - przesunięta o pół cyklu
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    right_y = tilt(right_phase, tilt_amp)
    
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
    """Wyświetla informacje debugowania o aktualnych kątach przegubów."""
    print("\n" + "=" * 50)
    print("AKTUALNE KĄTY PRZEGUBÓW:")

    for i in range(min(20, p.getNumJoints(robot))):
        angle = p.getJointState(robot, i)[0]
        print(f"Joint {i}: {angle:7.2f}°")

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

def updateRobot(joint_targets, max_rate_hz=50):
    global last_send_time
    now = time.time()

    # rate limiting (50 Hz domyślnie)
    if now - last_send_time < 1.0 / max_rate_hz:
        return

    last_send_time = now

    # ===== LEWA NOGA =====
    hip_roll_L   = 90 - math.degrees(joint_targets[0])   # servo 1
    hip_pitch_L  = 90 + math.degrees(joint_targets[1])   # servo 2
    knee_L       = 90 + math.degrees(joint_targets[3])   # servo 3
    ankle_L      = 90 + math.degrees(joint_targets[5])   # servo 4

    # ===== PRAWA NOGA =====
    hip_roll_R   = 90 - math.degrees(joint_targets[7])   # servo 5
    hip_pitch_R  = 90 - math.degrees(joint_targets[8])   # servo 6
    knee_R       = 90 + math.degrees(joint_targets[10])  # servo 7
    ankle_R      = 90 + math.degrees(joint_targets[12])  # servo 8
    
    print("Updating robot servos...")
    print(f"Left Leg Targets: Hip Roll: {(hip_roll_L):.2f}, Hip Pitch: {(hip_pitch_L):.2f}, Knee: {(knee_L):.2f}, Ankle: {(ankle_L):.2f}")
    print(f"Right Leg Targets: Hip Roll: {(hip_roll_R):.2f}, Hip Pitch: {(hip_pitch_R):.2f}, Knee: {(knee_R):.2f}, Ankle: {(ankle_R):.2f}")

    # ==== Wysyłanie do serw ====
    MoveServo(1, (hip_roll_L))
    MoveServo(2, (hip_pitch_L))
    MoveServo(3, (knee_L))
    MoveServo(4, (ankle_L))

    MoveServo(5, (hip_roll_R))
    MoveServo(6, (hip_pitch_R))
    MoveServo(7, (knee_R))
    MoveServo(8, (ankle_R))

def main():
    """Główna pętla symulacji."""
    robot = initialize_simulation()
    sliders = create_ui_sliders()

    # p.setGravity(0, 0, 0)
    # init_pose(robot)
        
    # for _ in range(int(2 * 240)):  # 240 Hz
    #     update_camera(robot, sliders)
    #     debug_info(robot)
    #     p.stepSimulation()
    #     time.sleep(1./240.)
    
    # p.setGravity(0, 0, -9.81)
    start_time = time.time()
    
    while True:
        current_time = time.time() - start_time
        
        # Odczytaj wartości ze sliderów
        swing_width = p.readUserDebugParameter(sliders['swing_width'])
        swing_height = p.readUserDebugParameter(sliders['swing_height'])
        tilt_amp = p.readUserDebugParameter(sliders['tilt_amplitude'])
        
        (left_x, left_y, left_z), (right_x, right_y, right_z) = get_trot_leg_positions(
            current_time, swing_width, swing_height, tilt_amp
        )

        try:
            left_targets = solve_ik_3d(left_x, left_y, left_z, "left", robot)
            right_targets = solve_ik_3d(right_x, right_y, right_z, "right", robot)
            
            joint_targets = combine_leg_targets(left_targets, right_targets)
            apply_joint_targets(robot, joint_targets)
        except ValueError as e:
            print(f"IK Error: {e}")
        
        updateRobot(joint_targets)

        # debug_info(robot)
        update_camera(robot, sliders)
        
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
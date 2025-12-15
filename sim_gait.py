import pybullet as p
import pybullet_data
import time
import numpy as np
import math
from servos import MoveServo

last_send_time = 0
# ========================================
# STAŁE KONFIGURACYJNE
# ========================================
TOTAL_HEIGHT = 0.302
SWING_WIDTH = 0.05
SWING_HEIGHT = 0.03
SWING_TIME = 0.35
Z_OFFSET = 0.02            # minimalne ugiecie nóg aby mieć zasięg w poziomie         
X_OFFSET = 0.0

LEFT_FOOT_TILT = 15
RIGHT_FOOT_TILT = 15
LEFT_HIP_TILT = 5
RIGHT_HIP_TILT = 5
HOLD_TIME = SWING_TIME * 1


# ========================================
# INICJALIZACJA
# ========================================
def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
    return robot

def create_debug_sliders():
    sliders = {
        'camera_distance': p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5),
        'camera_yaw': p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 90),
        'camera_pitch': p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0),
        'camera_height': p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0),
        'speed': p.addUserDebugParameter("  Predkosc chodu", 0.1, 5.0, 2.5),
        'swing_width': p.addUserDebugParameter("  Szerokosc kroku", 0.0, 0.05, 0),
        'swing_height': p.addUserDebugParameter("  Wysokosc kroku", 0.0, 0.05, 0),
        'tilt_gain': p.addUserDebugParameter("  Przechyl (Tilt)", 0.0, 1.0, 0)
    }
    return sliders

def trot_gait(phase: float, swing_width: float, swing_height: float):
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
    phase = phase % 1.0

    # Wyznaczenie granic
    p1 = HOLD_TIME               # Przechył w prawo
    p2 = 0.5                     # Koniec jazdy prawo → lewo
    p3 = 0.5 + HOLD_TIME         # Przechył w lewo
    # p4 = 1.0                   # Koniec jazdy lewo → prawo

    # --- FAZA 1: stałe przechylenie w prawo ---
    if phase < p1:
        return (
            0 - LEFT_FOOT_TILT,
            0 - RIGHT_FOOT_TILT,
            0,
            0 - RIGHT_HIP_TILT
        )

    # --- FAZA 2: przejazd prawo → lewo ---
    elif phase < p2:
        progress = (phase - p1) / (p2 - p1)
        return (
            0 - LEFT_FOOT_TILT + 2 * LEFT_FOOT_TILT * progress,
            0 - RIGHT_FOOT_TILT + 2 * RIGHT_FOOT_TILT * progress,
            0 + LEFT_HIP_TILT * progress,
            0 - RIGHT_HIP_TILT + RIGHT_HIP_TILT * progress
        )

    # --- FAZA 3: stałe przechylenie w lewo ---
    elif phase < p3:
        return (
            0 + LEFT_FOOT_TILT,
            0 + RIGHT_FOOT_TILT,
            0 + LEFT_HIP_TILT,
            0
        )

    # --- FAZA 4: przejazd lewo → prawo ---
    else:
        progress = (phase - p3) / (1.0 - p3)
        return (
            0 + LEFT_FOOT_TILT  - 2 * LEFT_FOOT_TILT * progress,
            0 + RIGHT_FOOT_TILT - 2 * RIGHT_FOOT_TILT * progress,
            0 + LEFT_HIP_TILT   - LEFT_HIP_TILT * progress,
            0 - RIGHT_HIP_TILT * progress
        )
    
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

def init_pose(robot):
    left_targets = solve_ik_3d(-0.0, 0.00, TOTAL_HEIGHT - Z_OFFSET, "left")
    right_targets = solve_ik_3d(0.0, 0.00, TOTAL_HEIGHT - Z_OFFSET, "right")

    left_targets[0] = np.radians(5)     # hip roll
    left_targets[5] = -np.radians(15)    # foot roll

    right_targets[7] = np.radians(5)   # hip roll
    right_targets[12] = -np.radians(15) # foot roll

    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        p.resetJointState(robot, i, joint_targets[i])

def calculate_leg_positions(phase, swing_width, swing_height):
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height)
    left_y = 0
    
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height)
    right_y = 0
    
    return (left_x, left_y, left_z), (right_x, right_y, right_z)

def combine_leg_targets(left_pos, right_pos, phase, tilt_gain):
    # --- pozycje stóp ---
    left_x, left_y, left_z = left_pos
    right_x, right_y, right_z = right_pos

    # --- bazowy tilt z funkcji czasowej ---
    left_foot_tilt, right_foot_tilt, left_hip_tilt, right_hip_tilt = tilt(phase)

    # --- SKALOWANIE SLIDEREM ---
    left_foot_tilt  *= tilt_gain
    right_foot_tilt *= tilt_gain
    left_hip_tilt   *= tilt_gain
    right_hip_tilt  *= tilt_gain

    # --- IK ---
    left_targets = solve_ik_3d(left_x,  0, left_z,  "left")
    right_targets = solve_ik_3d(right_x, 0, right_z, "right")

    # --- NADPISANIE TILTU ---
    # lewa noga
    left_targets[0] = -np.radians(left_hip_tilt)   # hip roll
    left_targets[5] =  np.radians(left_foot_tilt)  # foot roll

    # prawa noga
    right_targets[7] = -np.radians(right_hip_tilt) # hip roll
    right_targets[12] = np.radians(right_foot_tilt)# foot roll

    # --- złożenie nóg w jedną listę jointów ---
    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]

    return joint_targets

def apply_joint_targets(robot, joint_targets, force=10000):
    for jid in range(14):
        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=jid,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_targets[jid],
            targetVelocity=0,
            force=force,
            positionGain=0.3,     # Kp
            velocityGain=1.0,     # Kd (damping)
            maxVelocity=2.0
        )

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
    return {
        'speed': p.readUserDebugParameter(sliders['speed']),
        'swing_width': p.readUserDebugParameter(sliders['swing_width']),
        'swing_height': p.readUserDebugParameter(sliders['swing_height']),
        'tilt_gain': p.readUserDebugParameter(sliders['tilt_gain'])
    }

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
    robot = init_simulation()
    sliders = create_debug_sliders()
    phase = 0.0
    dt = 1./240.

    # p.setGravity(0, 0, 0)
    # # init_pose(robot)
        
    # for _ in range(int(0.2 * 240)):  # 240 Hz
    #     update_camera(robot, sliders)
    #     p.stepSimulation()
    #     time.sleep(1./240.)
    
    # p.setGravity(0, 0, -9.81)

    # --- Główna animacja ---
    while True:
        params = read_gait_parameters(sliders)

        phase += params['speed'] * dt
        phase = phase % 1.0

        left_pos, right_pos = calculate_leg_positions(
            phase, params['swing_width'], params['swing_height']
        )

        joint_targets = combine_leg_targets(left_pos, right_pos, phase, params['tilt_gain'])

        apply_joint_targets(robot, joint_targets)
        updateRobot(joint_targets)

        update_camera(robot, sliders)
        p.stepSimulation()
        time.sleep(dt)


if __name__ == "__main__":
    main()

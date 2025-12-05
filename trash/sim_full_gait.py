import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# ========================================
# INICJALIZACJA SYMULACJI
# ========================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Załaduj scenę
p.loadURDF("plane.urdf")
robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.290], useFixedBase=True)

# ========================================
# PARAMETRY CHODU
# ========================================
SWING_WIDTH = 0.08      # Szerokość kroku
SWING_HEIGHT = 0.05    # Wysokość uniesienia nogi
SWING_TIME = 0.5       # Procent czasu w fazie swing (0-1)
X_OFFSET = 0.0         # Przesunięcie w osi X
Z_OFFSET = 0.01       # Minimalna wysokość stopy

# Parametry Y (szerokość biodra)
LEFT_Y = 0.04
RIGHT_Y = -0.04

# ========================================
# SLIDERY DO KONTROLI
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 56)
camera_pitch_slider = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
camera_height_slider = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)

# Slidery parametrów chodu
speed_slider = p.addUserDebugParameter("  Predkosc chodu", 0.1, 5.0, 1.0)
swing_width_slider = p.addUserDebugParameter("  Szerokosc kroku", 0.05, 0.4, SWING_WIDTH)
swing_height_slider = p.addUserDebugParameter("  Wysokosc kroku", 0.01, 0.15, SWING_HEIGHT)

# ========================================
# PARAMETRY IK
# ========================================
LEFT_EE = 6   # End effector lewej nogi
RIGHT_EE = 13 # End effector prawej nogi

target_orientation = p.getQuaternionFromEuler([0, 0, 0])
num_joints = p.getNumJoints(robot)

# ========================================
# FUNKCJA TRAJEKTORII CHODU
# ========================================
def trot_gait(phase: float, swing_width: float, swing_height: float, swing_time: float, x_offset: float, z_offset: float):
    """Zwraca x,z stopy przy zadanej fazie 0 - 1"""
    half_w = swing_width / 2

    if phase < swing_time:
        # Faza swing - noga w powietrzu
        t = phase / swing_time
        angle = math.pi * (1 - t)
        x = half_w * math.cos(angle) + x_offset
        z = z_offset + swing_height * math.sin(angle)
    else:
        # Faza stance - noga na ziemi
        t = (phase - swing_time) / (1 - swing_time)
        x = half_w - swing_width * t + x_offset
        z = z_offset

    return x, z

# ========================================
# ZMIENNE STANU
# ========================================
phase = 0.0
start_time = time.time()

# ========================================
# GŁÓWNA PĘTLA SYMULACJI
# ========================================
while True:
    # ------------------------------------
    # AKTUALIZACJA FAZY
    # ------------------------------------
    current_time = time.time()
    speed = p.readUserDebugParameter(speed_slider)
    dt = 1./240.
    phase += speed * dt
    phase = phase % 1.0  # Zapętlenie 0-1
    
    # ------------------------------------
    # ODCZYT PARAMETRÓW Z SLIDERÓW
    # ------------------------------------
    swing_width = p.readUserDebugParameter(swing_width_slider)
    swing_height = p.readUserDebugParameter(swing_height_slider)
    
    # ------------------------------------
    # OBLICZENIE POZYCJI NÓG
    # ------------------------------------
    # Lewa noga - faza 0
    left_phase = phase
    left_x, left_z = trot_gait(left_phase, swing_width, swing_height, SWING_TIME, X_OFFSET, Z_OFFSET)
    left_target = [left_x, LEFT_Y, left_z]
    
    # Prawa noga - przesunięta o 180° (0.5 fazy)
    right_phase = (phase + 0.5) % 1.0
    right_x, right_z = trot_gait(right_phase, swing_width, swing_height, SWING_TIME, X_OFFSET, Z_OFFSET)
    right_target = [right_x, RIGHT_Y, right_z]

    # ------------------------------------
    # IK LEWA NOGA
    # ------------------------------------
    left_joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=LEFT_EE,
        targetPosition=left_target,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-6
    )

    # ------------------------------------
    # IK PRAWA NOGA
    # ------------------------------------
    right_joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=RIGHT_EE,
        targetPosition=right_target,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-6
    )

    # ------------------------------------
    # INICJALIZUJ TABLICĘ KĄTÓW
    # ------------------------------------
    joint_angles = [0.0] * num_joints

    # Przypisz kąty z lewego IK (jointy 0-5)
    for i in range(0, 6):
        joint_angles[i] = left_joint_positions[i]

    # Przypisz kąty z prawego IK (jointy 7-12)
    for i in range(6, 12):
        joint_angles[i + 1] = right_joint_positions[i]

    # ------------------------------------
    # WYDRUKUJ STAN (co 0.5s)
    # ------------------------------------
    if int((current_time - start_time) * 2) != int((current_time - start_time - dt) * 2):
        print("\n" + "="*60)
        print(f"FAZA: {phase:.3f} | LEWA: ({left_x:.3f}, {left_z:.3f}) | PRAWA: ({right_x:.3f}, {right_z:.3f})")
        print("="*60)

    # ------------------------------------
    # USTAWIENIE PRZEGUBÓW
    # ------------------------------------
    for i in range(num_joints):
        p.resetJointState(robot, i, joint_angles[i])

    # ------------------------------------
    # AKTUALIZACJA KAMERY
    # ------------------------------------
    camera_distance = p.readUserDebugParameter(camera_distance_slider)
    camera_yaw = p.readUserDebugParameter(camera_yaw_slider)
    camera_pitch = p.readUserDebugParameter(camera_pitch_slider)
    camera_height = p.readUserDebugParameter(camera_height_slider)

    robot_pos, _ = p.getBasePositionAndOrientation(robot)
    camera_target = [robot_pos[0], robot_pos[1], robot_pos[2] + camera_height]

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target
    )

    # ------------------------------------
    # KROK SYMULACJI
    # ------------------------------------
    p.stepSimulation()
    time.sleep(dt)
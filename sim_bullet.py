import math
import pybullet as p
import pybullet_data
import time
import numpy as np
from servos import MoveServo

# ========================================
# KONFIGURACJA STAŁYCH
# ========================================
LEFT_EE = 6
RIGHT_EE = 13

# ========================================
# FUNKCJE POMOCNICZE
# ========================================
def init_simulation():
    """Inicjalizacja środowiska + ładowanie robota"""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=True)
    return robot

def configure_dynamics(robot):
    """Ustawienia tłumienia i tarcia"""
    num_joints = p.getNumJoints(robot)

    for j in range(num_joints):
        p.changeDynamics(robot, j,
                         linearDamping=0.8,
                         angularDamping=0.8,
                         jointDamping=0.8)

    for ee in [LEFT_EE, RIGHT_EE]:
        p.changeDynamics(robot, ee,
                         lateralFriction=1.3,
                         spinningFriction=0.1,
                         rollingFriction=0.1)

def create_sliders():
    """Tworzy wszystkie slidery i zwraca je w słowniku"""

    sliders = {}

    sliders["cam_dist"] = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
    sliders["cam_yaw"] = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 56)
    sliders["cam_pitch"] = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
    sliders["cam_h"] = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)

    sliders["Lx"] = p.addUserDebugParameter("  L - X", -0.3, 0.3, 0.0)
    sliders["Ly"] = p.addUserDebugParameter("  L - Y", -0.2, 0.2, 0.04)
    sliders["Lz"] = p.addUserDebugParameter("  L - Z", 0.006, 0.35, 0.006)

    sliders["Rx"] = p.addUserDebugParameter("  P - X", -0.3, 0.3, 0.0)
    sliders["Ry"] = p.addUserDebugParameter("  P - Y", -0.2, 0.2, -0.04)
    sliders["Rz"] = p.addUserDebugParameter("  P - Z", 0.006, 0.35, 0.006)

    return sliders

def get_leg_targets(sliders):
    """Czyta wartości pozycji nóg ze sliderów"""

    left = [p.readUserDebugParameter(sliders["Lx"]),
            p.readUserDebugParameter(sliders["Ly"]),
            p.readUserDebugParameter(sliders["Lz"])]

    right = [p.readUserDebugParameter(sliders["Rx"]),
             p.readUserDebugParameter(sliders["Ry"]),
             p.readUserDebugParameter(sliders["Rz"])]

    return left, right

def compute_IK(robot, left_target, right_target):
    """Oblicza IK i scala obie nogi w jedną tablicę jointów"""
    num_joints = p.getNumJoints(robot)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    left_ik = p.calculateInverseKinematics(
        robot, LEFT_EE, left_target, target_orientation)

    right_ik = p.calculateInverseKinematics(
        robot, RIGHT_EE, right_target, target_orientation)

    joint_angles = [0] * num_joints

    for i in range(6):  # lewa noga
        joint_angles[i] = left_ik[i]

    for i in range(6, 12):  # prawa noga -> przesunięcie o 1
        joint_angles[i + 1] = right_ik[i]

    return joint_angles

def apply_joint_angles(robot, joint_angles):
    """Ustawiający wszystkie stawy"""
    for i, angle in enumerate(joint_angles):
        p.resetJointState(robot, i, angle)

def update_camera(robot, sliders):
    """Kontrola kamery sliderami"""
    dist = p.readUserDebugParameter(sliders["cam_dist"])
    yaw = p.readUserDebugParameter(sliders["cam_yaw"])
    pitch = p.readUserDebugParameter(sliders["cam_pitch"])
    height = p.readUserDebugParameter(sliders["cam_h"])

    pos, _ = p.getBasePositionAndOrientation(robot)
    target = [pos[0], pos[1], pos[2] + height]

    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

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

# ========================================
# GŁÓWNY PROGRAM
# ========================================

def main():
    robot = init_simulation()
    configure_dynamics(robot)
    
    sliders = create_sliders()

    while True:
        left_target, right_target = get_leg_targets(sliders)
        joint_angles = compute_IK(robot, left_target, right_target)

        # Wyświetlenie jointów
        left_leg_angles = joint_angles[:7]
        right_leg_angles = joint_angles[7:14]
        print("Lewa noga:  ", ["{:.3f}".format(a) for a in left_leg_angles])
        print("Prawa noga: ", ["{:.3f}".format(a) for a in right_leg_angles])        

        apply_joint_angles(robot, joint_angles)

        hip_roll_L   = 90 - math.degrees(joint_angles[0])   # servo 1
        hip_pitch_L  = 90 + math.degrees(joint_angles[1])   # servo 2
        knee_L       = 90 + math.degrees(joint_angles[3])   # servo 3
        ankle_L      = 90 + math.degrees(joint_angles[5])   # servo 4

        # ===== PRAWA NOGA =====
        hip_roll_R   = 90 - math.degrees(joint_angles[7])   # servo 5
        hip_pitch_R  = 90 - math.degrees(joint_angles[8])   # servo 6
        knee_R       = 90 + math.degrees(joint_angles[10])  # servo 7
        ankle_R      = 90 + math.degrees(joint_angles[12])  # servo 8
        # ==== Wysyłanie do serw ====
        MoveServo(1, (hip_roll_L))
        MoveServo(2, (hip_pitch_L))
        MoveServo(3, (knee_L))
        MoveServo(4, (ankle_L))

        MoveServo(5, (hip_roll_R))
        MoveServo(6, (hip_pitch_R))
        MoveServo(7, (knee_R))
        MoveServo(8, (ankle_R))
        update_camera(robot, sliders)

        p.stepSimulation()
        time.sleep(1 / 240.0)


if __name__ == "__main__":
    main()

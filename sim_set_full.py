import pybullet as p
import pybullet_data
import time
import numpy as np

# ========================================
# INICJALIZACJA SYMULACJI
# ========================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Załaduj scenę
p.loadURDF("plane.urdf")
robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=True)

# ========================================
# SLIDERY DO KONTROLI KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 56)
camera_pitch_slider = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
camera_height_slider = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)

# ========================================
# SLIDERY POZYCJI LEWEJ NOGI
# ========================================
left_x_slider = p.addUserDebugParameter("  L - X (przod/tyl)", -0.3, 0.3, 0.0)
left_y_slider = p.addUserDebugParameter("  L - Y (bok)", -0.2, 0.2, 0.04)
left_z_slider = p.addUserDebugParameter("  L - Z (gora/dol)", 0.006, 0.35, 0.006)

# ========================================
# SLIDERY POZYCJI PRAWEJ NOGI
# ========================================
right_x_slider = p.addUserDebugParameter("  P - X (przod/tyl)", -0.3, 0.3, 0.0)
right_y_slider = p.addUserDebugParameter("  P - Y (bok)", -0.2, 0.2, -0.04)
right_z_slider = p.addUserDebugParameter("  P - Z (gora/dol)", 0.006, 0.35, 0.006)

# ========================================
# PARAMETRY IK
# ========================================
LEFT_EE = 6   # End effector lewej nogi
RIGHT_EE = 13 # End effector prawej nogi

num_joints = p.getNumJoints(robot)

# ========================================
# GŁÓWNA PĘTLA SYMULACJI
# ========================================
while True:
    # ------------------------------------
    # ODCZYT POZYCJI Z SLIDERÓW
    # ------------------------------------
    left_target = [
        p.readUserDebugParameter(left_x_slider),
        p.readUserDebugParameter(left_y_slider),
        p.readUserDebugParameter(left_z_slider)
    ]
    
    right_target = [
        p.readUserDebugParameter(right_x_slider),
        p.readUserDebugParameter(right_y_slider),
        p.readUserDebugParameter(right_z_slider)
    ]

    # ------------------------------------
    # IK LEWA NOGA
    # ------------------------------------
    left_joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=LEFT_EE,
        targetPosition=left_target,
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
        maxNumIterations=100,
        residualThreshold=1e-6
    )

    # ------------------------------------
    # INICJALIZUJ TABLICĘ KĄTÓW
    # ------------------------------------
    joint_angles = [0.0] * num_joints

    # Przypisz kąty z lewego IK
    # left_joint_positions[0-5] → joint_angles[0-5] (lewa noga)
    # left_joint_positions[6-11] → ignorujemy (to prawa noga z tego samego wywołania IK)
    # Joint 6 jest FIXED - nie trzeba ustawiać
    for i in range(0, 6):  # Tylko ruchome jointy lewej nogi (0-5)
        joint_angles[i] = left_joint_positions[i]

    # Przypisz kąty z prawego IK
    # right_joint_positions[0-5] → ignorujemy (to lewa noga z tego samego wywołania IK)
    # right_joint_positions[6-11] → joint_angles[7-12] (prawa noga, z przesunięciem!)
    # Joint 13 jest FIXED - nie trzeba ustawiać
    for i in range(6, 12):  # Pozycje 6-11 w tablicy IK
        joint_angles[i + 1] = right_joint_positions[i]  # Mapuj na jointy 7-12


    # ------------------------------------
    # WYDRUKUJ TABLICĘ JOINT_ANGLES
    # ------------------------------------
    print("\n" + "="*60)
    print("TABLICA JOINT_ANGLES (po scaleniu)")
    print("="*60)

    print("\nLEWA NOGA (jointy 0-6):")
    print("-"*40)
    for i in range(0, min(7, len(joint_angles))):
        deg = np.degrees(joint_angles[i])
        print(f"  joint_angles[{i:2d}] = {joint_angles[i]:8.4f} rad  ({deg:8.2f}°)")

    print("\nPRAWA NOGA (jointy 7-13):")
    print("-"*40)
    for i in range(7, min(14, len(joint_angles))):
        deg = np.degrees(joint_angles[i])
        print(f"  joint_angles[{i:2d}] = {joint_angles[i]:8.4f} rad  ({deg:8.2f}°)")

    print("\n" + "="*60)

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
    time.sleep(1./240.)
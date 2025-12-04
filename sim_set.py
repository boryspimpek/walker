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
robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.325], useFixedBase=True)

# ========================================
# SLIDERY DO KONTROLI KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, -0)
camera_pitch_slider = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
camera_height_slider = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)

# ========================================
# PARAMETRY DOCELOWE IK
# ========================================
target_x = 0.08
target_y = 0.08
target_z = 0.1

ee_index = 6

# ========================================
# GŁÓWNA PĘTLA SYMULACJI
# ========================================
while True:
    # ------------------------------------
    # OBLICZ INVERSE KINEMATICS
    # ------------------------------------
    target_position = [target_x, target_y, target_z]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=ee_index,
        targetPosition=target_position,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    
    # ------------------------------------
    # WYŚWIETL OBLICZONE KĄTY
    # ------------------------------------
    print("=" * 50)
    print("OBLICZONE KĄTY Z IK:")
    for i, angle in enumerate(joint_positions):
        print(f"  Przegub {i}: {angle:7.4f} rad ({np.degrees(angle):7.2f}°)")
    
    # ------------------------------------
    # USTAW PRZEGUBY NATYCHMIAST
    # ------------------------------------
    for i in range(len(joint_positions)):
        p.resetJointState(robot, i, joint_positions[i])
        
    # ------------------------------------
    # AKTUALIZUJ KAMERĘ
    # ------------------------------------
    camera_distance = p.readUserDebugParameter(camera_distance_slider)
    camera_yaw = p.readUserDebugParameter(camera_yaw_slider)
    camera_pitch = p.readUserDebugParameter(camera_pitch_slider)
    camera_height = p.readUserDebugParameter(camera_height_slider)
    
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
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
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
robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.291], useFixedBase=True)

# ========================================
# CONSTRAINT GEAR - POŁĄCZENIE PRZEGUBÓW
# ========================================
gear_constraint = p.createConstraint(
    parentBodyUniqueId=robot,
    parentLinkIndex=1,          # Joint 1 (parent)
    childBodyUniqueId=robot,
    childLinkIndex=2,           # Joint 2 (child)
    jointType=p.JOINT_GEAR,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0]
)

# Ustaw współczynnik przekładni: 1 = przeciwny kierunek
p.changeConstraint(gear_constraint, gearRatio=-1, maxForce=100000, erp=1.0)

# ========================================
# SLIDERY DO KONTROLI KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 0)
camera_pitch_slider = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
camera_height_slider = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)

# ========================================
# PARAMETRY DOCELOWE IK
# ========================================
target_x_slider = p.addUserDebugParameter("  Pozycja X", -0.5, 0.5, 0.0)
target_y_slider = p.addUserDebugParameter("  Pozycja Y", -0.5, 0.5, 0.0)
target_z_slider = p.addUserDebugParameter("  Pozycja Z", 0.0, 0.5, 0.0)

# ========================================
# GŁÓWNA PĘTLA SYMULACJI
# ========================================
frame_count = 0
while True:
    frame_count += 1
    
    # ------------------------------------
    # OBLICZ INVERSE KINEMATICS
    # ------------------------------------
    target_x = p.readUserDebugParameter(target_x_slider)
    target_y = p.readUserDebugParameter(target_y_slider)
    target_z = p.readUserDebugParameter(target_z_slider)

    target_position = [target_x, target_y, target_z]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=6,
        targetPosition=target_position,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    
    # ------------------------------------
    # KONTROLA SILNIKÓW
    # ------------------------------------
    num_joints = p.getNumJoints(robot)
    
    for i in range(min(len(joint_positions), num_joints)):
        if i == 2:
            # Joint 2: WYŁĄCZ aktywny kontroler, aby gear mógł działać
            p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0  # Brak siły = gear constraint przejmuje kontrolę
            )
        else:
            # Wszystkie inne joiny: normalna kontrola pozycyjna
            p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_positions[i],
                force=2000,
                positionGain=0.8,      
                velocityGain=1, 
                maxVelocity=10  
            )
        
    # ------------------------------------
    # WYŚWIETL CO 60 KLATEK
    # ------------------------------------
    if frame_count % 60 == 0:
        print("=" * 50)
        print("AKTUALNE KĄTY PRZEGUBÓW:")
        for i in range(min(40, num_joints)):
            joint_state = p.getJointState(robot, i)
            actual_angle = joint_state[0]
            print(f"  Przegub {i}: {actual_angle:7.4f} rad ({np.degrees(actual_angle):7.2f}°)")
        
        # Sprawdź relację gear
        joint1_angle = p.getJointState(robot, 1)[0]
        joint2_angle = p.getJointState(robot, 2)[0]
        print(f"\n  WERYFIKACJA GEAR:")
        print(f"  Joint 1: {np.degrees(joint1_angle):7.2f}°")
        print(f"  Joint 2: {np.degrees(joint2_angle):7.2f}° (oczekiwane: {-np.degrees(joint1_angle):7.2f}°)")
        print(f"  Suma: {np.degrees(joint1_angle + joint2_angle):7.2f}° (powinno być ~0°)")
    
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
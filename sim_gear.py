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
robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.27], useFixedBase=True)

# ========================================
# UTWÓRZ GEAR CONSTRAINT
# ========================================
gear_constraint = p.createConstraint(
    parentBodyUniqueId=robot,
    parentLinkIndex=0,
    childBodyUniqueId=robot,
    childLinkIndex=1,
    jointType=p.JOINT_GEAR,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)

# Ustaw przełożenie i siłę
p.changeConstraint(gear_constraint, gearRatio=1, maxForce=100000, erp=1.0)

# ========================================
# SLIDERY DO KONTROLI KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("Odległość kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("Obrót kamery (Yaw)", -180, 180, 0)
camera_pitch_slider = p.addUserDebugParameter("Nachylenie kamery (Pitch)", -89, 89, 0)
camera_height_slider = p.addUserDebugParameter("Wysokość kamery", -1.0, 1.0, 0.0)

# ========================================
# PARAMETRY DOCELOWE IK
# ========================================
target_x = 0.14806
target_y = 0
target_z = 0.27 - 0.1638919

ee_index = 5

# ========================================
# GŁÓWNA PĘTLA SYMULACJI
# ========================================
frame_count = 0
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
    
    # Steruj Joint 0 - Joint 1 zostanie zsynchronizowany przez gear constraint
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=joint_positions[0],
        force=500,
        maxVelocity=10
    )
    
    # Wyłącz kontroler dla Joint 1, aby gear constraint mógł działać
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=1,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0  # Zero force = brak aktywnego kontrolera
    )
    
    # Steruj pozostałymi jointami
    for i in range(2, len(joint_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[i],
            force=500,
            maxVelocity=10
        )
    
    # ------------------------------------
    # WYŚWIETL CO 60 KLATEK
    # ------------------------------------
    if frame_count % 60 == 0:
        print("=" * 50)
        print("AKTUALNE KĄTY PRZEGUBÓW:")
        num_joints = p.getNumJoints(robot)
        for i in range(min(4, num_joints)):
            joint_state = p.getJointState(robot, i)
            actual_angle = joint_state[0]
            print(f"  Przegub {i}: {actual_angle:7.4f} rad ({np.degrees(actual_angle):7.2f}°)")
        
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
    frame_count += 1
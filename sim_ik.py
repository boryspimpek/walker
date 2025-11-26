import pybullet as p
import pybullet_data
import time
import numpy as np

URDF = "walker.urdf"

# === INICJALIZACJA ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# Podłoże
plane_id = p.loadURDF("plane.urdf")

# Robot
robot_id = p.loadURDF(URDF, basePosition=[0, 0, 0.5], useFixedBase=True)

# === IDENTYFIKACJA STAWÓW ===
num_joints = p.getNumJoints(robot_id)
controllable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    if info[2] == p.JOINT_REVOLUTE:
        controllable_joints.append(i)

EE_INDEX = num_joints - 1  # Stopa (right_foot_1)

print(f"Stawy do sterowania: {controllable_joints}")
print(f"End-effector: {EE_INDEX}")

# === SLIDERY - POZYCJA + ORIENTACJA ===
# Pozycja
target_x = p.addUserDebugParameter("Target X", -0.3, 0.3, 0.0)
target_y = p.addUserDebugParameter("Target Y", -0.3, 0.3, 0.0)
target_z = p.addUserDebugParameter("Target Z", 0.0, 0.8, 0.0)

# Orientacja (kwaterniony lub Euler)
roll = p.addUserDebugParameter("Roll", -3.14, 3.14, 0.0)
pitch = p.addUserDebugParameter("Pitch", -3.14, 3.14, 0.0)
yaw = p.addUserDebugParameter("Yaw", -3.14, 3.14, 0.0)

while True:
    # Odczyt pozycji docelowej
    tx = p.readUserDebugParameter(target_x)
    ty = p.readUserDebugParameter(target_y)
    tz = p.readUserDebugParameter(target_z)
    target_pos = [tx, ty, tz]
    
    # Odczyt orientacji (Euler -> Kwaternion)
    r = p.readUserDebugParameter(roll)
    p_val = p.readUserDebugParameter(pitch)
    y = p.readUserDebugParameter(yaw)
    
    # Konwersja Euler -> Kwaternion
    target_orn = p.getQuaternionFromEuler([r, p_val, y])
    
    # INVERSE KINEMATICS z orientacją
    joint_positions = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=EE_INDEX,
        targetPosition=target_pos,
        targetOrientation=target_orn,  # Dodana orientacja!
        maxNumIterations=200,
        residualThreshold=0.001
    )

    # Sterowanie stawami
    for joint_idx, target_angle in zip(controllable_joints, joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            targetVelocity=0,  # Chcemy zatrzymać się w pozycji docelowej
            force=300,       # Maksymalna siła
            # positionGain=pos_gain,  # Jak agresywnie dążyć do pozycji
            # velocityGain=vel_gain   # Jak tłumić oscylacje
        )

    # Wizualizacja celu
    p.addUserDebugLine(target_pos, [tx, ty, tz+0.1], [1, 0, 0], 2)
    
    p.stepSimulation()
    time.sleep(1/240)
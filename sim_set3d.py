import math
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

p.loadURDF("plane.urdf")
robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.302], useFixedBase=True)
num_joints = p.getNumJoints(robot)

# ========================================
# SLIDERY KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Obrot", -180, 180, 0)
camera_pitch_slider = p.addUserDebugParameter("  Pitch", -89, 89, -5)
camera_height_slider = p.addUserDebugParameter("  Wysokosc", -1.0, 1.0, 0.0)

# ========================================
# SLIDERY DLA STÓP (3D)
# ========================================
x_slider = p.addUserDebugParameter("  X target (przod-tyl)", -0.15, 0.15, 0.0)
y_slider = p.addUserDebugParameter("  Y target (lewo-prawo)", -0.134, 0.134, 0.0)
z_slider = p.addUserDebugParameter("  Z target (gora-dol)", 0, 0.134, 0.067)

# ========================================
# IK — Funkcja inverse kinematics 3D
# ========================================
def solve_ik_3d(x, y, z, elbow_up=False):
    L1, L2 = 0.067, 0.067
    # --- Hip Roll ---
    theta1 = math.atan2(y, z)

    # Project into XZ plane after roll
    z_prime = math.sqrt(y**2 + z**2)
    x_prime = x

    # Distance in pitch plane
    D = math.sqrt(x_prime**2 + z_prime**2)

    # Check reach limit
    if D > (L1 + L2):
        raise ValueError("Target position outside reachable workspace!")

    # Knee angle - law of cosines
    theta3 = math.pi - math.acos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))

    # Hip pitch
    alpha = math.atan2(z_prime, x_prime)
    beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    theta2 = alpha - beta

    # KROK 6: Mapowanie na joiny robota
    joint_targets = [0.0] * 7
    
    joint_targets[0] = theta1 
    joint_targets[1] = - (1.5788 - theta2)
    joint_targets[2] = - (1.5788 - theta2)
    joint_targets[3] = theta3 - (1.5788 - theta2)
    joint_targets[4] = abs(theta2 - 1.5788) - abs(theta3)
    joint_targets[5] = - theta1  
    joint_targets[6] = 0  # fixed joint
    
    print(f"IK Targets: x={x:.3f}, y={y:.3f}, z={z:.3f} => theta1={math.degrees(theta1):.3f}, theta2={math.degrees(theta2):.3f}, theta3={math.degrees(theta3):.3f}")
    print("Joint Targets (deg): [" + ", ".join(f"{math.degrees(j):.3f}" for j in joint_targets) + "]")
    return joint_targets

# ========================================
# GŁÓWNA PĘTLA
# ========================================

while True:
    x_target = p.readUserDebugParameter(x_slider)
    y_target = p.readUserDebugParameter(y_slider)
    z_target = p.readUserDebugParameter(z_slider)
    
    joint_targets = solve_ik_3d(x_target, y_target, z_target)

    for jid in range(num_joints):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets[jid],
            force=500
        )
    
    cam_dist = p.readUserDebugParameter(camera_distance_slider)
    cam_yaw = p.readUserDebugParameter(camera_yaw_slider)
    cam_pitch = p.readUserDebugParameter(camera_pitch_slider)
    cam_height = p.readUserDebugParameter(camera_height_slider)
    
    base_pos, _ = p.getBasePositionAndOrientation(robot)
    cam_target = [base_pos[0], base_pos[1], base_pos[2] + cam_height]
    
    p.resetDebugVisualizerCamera(
        cameraDistance=cam_dist,
        cameraYaw=cam_yaw,
        cameraPitch=cam_pitch,
        cameraTargetPosition=cam_target
    )
    
    p.stepSimulation()
    time.sleep(1./240.)
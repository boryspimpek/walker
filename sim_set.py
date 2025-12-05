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
robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
num_joints = p.getNumJoints(robot)

# ========================================
# SLIDERY KAMERY
# ========================================
camera_distance_slider = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
camera_yaw_slider = p.addUserDebugParameter("  Yaw", -180, 180, 45)
camera_pitch_slider = p.addUserDebugParameter("  Pitch", -89, 89, -20)
camera_height_slider = p.addUserDebugParameter("  Wysokosc", -1.0, 1.0, 0.0)

# ========================================
# SLIDERY DLA STÓP
# ========================================
x_slider = p.addUserDebugParameter("X target", -0.15, 0.15, 0.0)
z_slider = p.addUserDebugParameter("Z target", -0.134, -0.01, -0.134)

# ========================================
# IK — Funkcja inverse kinematics
# ========================================
def solve_ik_2d(x, z, elbow_up=False):
    l1 = 0.067
    l2 = 0.067

    cos_theta2 = (x*x + z*z - l1*l1 - l2*l2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(z, x) - math.atan2(k2, k1)

    return theta1, theta2

# ========================================
# GŁÓWNA PĘTLA
# ========================================
while True:
    x_target = p.readUserDebugParameter(x_slider)
    z_target = p.readUserDebugParameter(z_slider)

    theta1, theta3 = solve_ik_2d(x_target, z_target)

    joint_targets = [0.0] * num_joints
    joint_targets[1] = -theta1 - 1.57
    joint_targets[2] = -theta1 - 1.57
    joint_targets[3] = -theta3 - theta1 - 1.57
    joint_targets[4] = theta3 + theta1 + 1.57

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

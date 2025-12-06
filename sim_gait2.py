import pybullet as p
import pybullet_data
import math
import numpy as np
import time

# ========================================
# PARAMETRY CHODU
# ========================================
SWING_WIDTH = 0.022
SWING_HEIGHT = 0.03
SWING_TIME = 0.5
Z_OFFSET = 0.011         # początkowe ugięcie nóg
X_OFFSET = 0.0
GAIT_SPEED = 0.8  # cykle/s

def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", [0.011, 0, 0.302], useFixedBase=False)

    return robot

def wait_for_stabilization(robot, camera_ui, duration_seconds=1):
    initial_angles = [
        -0.00,
        -0.30,
        -0.31,
         0.49,
        -0.49,
         0.00,
         0.00,
         0.00,
        -0.31,
         0.31,
        -0.49,
         0.49,
         0.00,
         0.00,
    ]

    p.setGravity(0, 0, 0)
    
    for i, angle in enumerate(initial_angles):
        p.resetJointState(robot, i, angle)
    
    frames = int(240 * duration_seconds)
    for _ in range(frames):
        update_camera(robot, camera_ui)
        p.stepSimulation()
        time.sleep(1/240)
    
    p.setGravity(0, 0, -9.81)

def solve_ik(x_target, z_target, leg, elbow_up=False):
    x = x_target
    z = z_target - 0.090 - 0.077 + 0.033
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

    joint_targets = [0.0] * 14  # zawsze mamy 14 wartości
    if leg == "left":
        joint_targets[0] = 0
        joint_targets[1] = -theta1 - 1.57
        joint_targets[2] = -theta1 - 1.57
        joint_targets[3] = -theta2 - theta1 - 1.57
        joint_targets[4] = theta2 + theta1 + 1.57
        joint_targets[5] = 0
        joint_targets[6] = 0
    else: 
        joint_targets[7] = 0
        joint_targets[8] = -theta1 - 1.57
        joint_targets[9] = theta1 + 1.57
        joint_targets[10] = theta2 + theta1 + 1.57
        joint_targets[11] = -theta2 - theta1 - 1.57
        joint_targets[12] = 0
        joint_targets[13] = 0
    return joint_targets

def trot_gait(phase: float):
    """Zwraca x,z stopy przy zadanej fazie 0 - 1"""
    half_w = SWING_WIDTH / 2

    if phase < SWING_TIME:
        t = phase / SWING_TIME
        angle = math.pi * (1 - t)
        x = half_w * math.cos(angle) + X_OFFSET  
        z = Z_OFFSET + SWING_HEIGHT * math.sin(angle)
    else:
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = half_w - SWING_WIDTH * t + X_OFFSET  
        z = Z_OFFSET

    return x, z

def combine_leg_targets(left_targets, right_targets):
    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    return joint_targets

def apply_joint_targets(robot, joint_targets, force=500):
    for jid in range(14):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets[jid],
            force=force
        )

def init_camera_ui():
    return {
        "dist": p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.6),
        "yaw": p.addUserDebugParameter("  Obrot kamery Yaw", -180, 180, 0),
        "pitch": p.addUserDebugParameter("  Nachylenie Pitch", -89, 89, 0),
        "height": p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0),
    }

def update_camera(robot, ui):
    robot_pos, _ = p.getBasePositionAndOrientation(robot)

    p.resetDebugVisualizerCamera(
        cameraDistance=p.readUserDebugParameter(ui["dist"]),
        cameraYaw=p.readUserDebugParameter(ui["yaw"]),
        cameraPitch=p.readUserDebugParameter(ui["pitch"]),
        cameraTargetPosition=[
            robot_pos[0],
            robot_pos[1],
            robot_pos[2] + p.readUserDebugParameter(ui["height"])
        ]
    )

def debug_info(robot):
    print("\n" + "=" * 50)
    print("AKTUALNE KĄTY PRZEGUBÓW:")

    for i in range(min(20, p.getNumJoints(robot))):
        angle = p.getJointState(robot, i)[0]
        print(f"Joint {i}: {np.degrees(angle):7.2f}°")

def main():
    robot = init_simulation()
    camera_ui = init_camera_ui()

    wait_for_stabilization(robot, camera_ui)

    frame = 0
    while True:
        frame += 1
        phase = (frame * GAIT_SPEED / 240.0) % 1.0

        x, z = trot_gait(phase)
        left_targets  = solve_ik(x, z, "left")
        right_targets = solve_ik(x, z, "right")

        joint_targets = combine_leg_targets(left_targets, right_targets)

        apply_joint_targets(robot, joint_targets)

        debug_info(robot)
        update_camera(robot, camera_ui)

        p.stepSimulation()
        time.sleep(1/240)


if __name__ == "__main__":
    main()

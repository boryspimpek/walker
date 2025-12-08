import pybullet as p
import pybullet_data
import math
import numpy as np
import time

# ========================================
# PARAMETRY CHODU
# ========================================
TOTAL_HEIGHT = 0.302
SWING_WIDTH = 0.1
SWING_HEIGHT = 0.03
SWING_TIME = 0.5
Z_OFFSET = 0.015            # minimalne ugiecie nóg aby mieć zasięg w poziomie         
X_OFFSET = 0.0
GAIT_SPEED = 0.8            # cykle/s

def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", [0.011, 0, TOTAL_HEIGHT], useFixedBase=True)

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
    
    # for i, angle in enumerate(initial_angles):
    #     p.resetJointState(robot, i, angle)
    
    frames = int(240 * duration_seconds)
    for _ in range(frames):
        update_camera(robot, camera_ui)
        p.stepSimulation()
        time.sleep(1/240)
    
    p.setGravity(0, 0, -9.81)

def solve_ik_3d(x, y, zt, leg, elbow_up=False):
    z = zt - 0.128
    l1, l2, l3 = 0.04, 0.067, 0.067
    hip_roll = np.arctan2(y, z)
    
    D = np.sqrt(y**2 + z**2)
    print(f"D: {D:.3f}")
    
    r = np.sqrt(x**2 + (D-l1)**2)
    print(f"r: {r:.3f}")
    
    cos_knee = (l2**2 + l3**2 - r**2) / (2 * l2 * l3)
    print(f"cos_knee: {cos_knee:.3f}")
    
    if cos_knee < -1 or cos_knee > 1:
        raise ValueError(f"Pozycja ({x:.3f}, {y:.3f}, {z:.3f}) jest poza zasięgiem nogi")
    
    knee_pitch = np.pi - np.arccos(cos_knee)

    alpha = np.arctan2(x, (D-l1))
    cos_beta = (l2**2 + r**2 - l3**2) / (2 * l2 * r)
    beta = np.arccos(np.clip(cos_beta, -1, 1))
    hip_pitch = -(alpha + beta)

    print(f"Noga {leg} - Obliczone kąty stawów:")
    print(f"  Hip Roll:  {np.degrees(hip_roll):7.2f}°")
    print(f"  Hip Pitch: {np.degrees(hip_pitch):7.2f}°")
    print(f"  Knee Pitch: {np.degrees(knee_pitch):7.2f}°")
    
    # Mapowanie na joiny robota
    joint_targets = [0.0] * 14
    if leg == "left":
        joint_targets[0] = hip_roll 
        joint_targets[1] = hip_pitch
        joint_targets[2] = hip_pitch
        joint_targets[3] = knee_pitch + hip_pitch
        joint_targets[4] = -(knee_pitch + hip_pitch)
        joint_targets[5] = -hip_roll
        joint_targets[6] = 0  # fixed joint
    else:  # right
        joint_targets[7] = hip_roll 
        joint_targets[8] = hip_pitch
        joint_targets[9] = -hip_pitch
        joint_targets[10] = -(knee_pitch + hip_pitch)
        joint_targets[11] = knee_pitch + hip_pitch
        joint_targets[12] = -hip_roll
        joint_targets[13] = 0  # fixed joint

    return joint_targets

def trot_gait(phase: float):
    """Zwraca x,z stopy przy zadanej fazie 0 - 1"""
    half_w = SWING_WIDTH / 2

    if phase < SWING_TIME:
        t = phase / SWING_TIME
        angle = math.pi * t  # Zmiana: t zamiast (1 - t)
        x = -half_w + SWING_WIDTH * t + X_OFFSET  # Liniowy ruch od tyłu do przodu
        z = (TOTAL_HEIGHT - Z_OFFSET) - SWING_HEIGHT * math.sin(angle)  # Minus - łuk w górę
    else:
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = half_w - SWING_WIDTH * t + X_OFFSET  
        z = TOTAL_HEIGHT - Z_OFFSET

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
        y = 0.0
        left_targets  = solve_ik_3d(x, y, z, "left")
        right_targets = solve_ik_3d(x, y, z, "right")

        joint_targets = combine_leg_targets(left_targets, right_targets)

        apply_joint_targets(robot, joint_targets)

        debug_info(robot)
        update_camera(robot, camera_ui)

        p.stepSimulation()
        time.sleep(1/240)


if __name__ == "__main__":
    main()

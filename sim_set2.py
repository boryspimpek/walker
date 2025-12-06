import math
import pybullet as p
import pybullet_data
import time
import numpy as np

# ========================================
# INICJALIZACJA SYMULACJI
# ========================================

def initialize_simulation():
    """Inicjalizuje symulację PyBullet z GUI i ustawieniami podstawowymi."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
    
    return robot

def create_ui_sliders():
    """Tworzy slidery interfejsu użytkownika do kontroli kamery i celów IK."""
    sliders = {
        'camera_distance': p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5),
        'camera_yaw': p.addUserDebugParameter("  Obrot", -180, 180, 45),
        'camera_pitch': p.addUserDebugParameter("  Pitch", -89, 89, -20),
        'camera_height': p.addUserDebugParameter("  Wysokosc", -1.0, 1.0, 0.0),
        'x_target': p.addUserDebugParameter("  X target", -0.15, 0.15, 0.0),
        'z_target': p.addUserDebugParameter("  Z target", -0.134, 0.15, 0.0)
    }
    return sliders

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

    joint_targets = [0.0] * 14
    
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

def combine_leg_targets(left_targets, right_targets):
    """Łączy cele dla lewej i prawej nogi w jeden wektor."""
    joint_targets = [0.0] * 14
    for i in range(14):
        joint_targets[i] = left_targets[i] if left_targets[i] != 0 else right_targets[i]
    return joint_targets

def apply_joint_targets(robot, joint_targets, force=500):
    """Aplikuje docelowe pozycje do przegubów robota."""
    for jid in range(14):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets[jid],
            force=force
        )

def update_camera(robot, sliders):
    """Aktualizuje pozycję kamery na podstawie sliderów."""
    cam_dist = p.readUserDebugParameter(sliders['camera_distance'])
    cam_yaw = p.readUserDebugParameter(sliders['camera_yaw'])
    cam_pitch = p.readUserDebugParameter(sliders['camera_pitch'])
    cam_height = p.readUserDebugParameter(sliders['camera_height'])

    base_pos, _ = p.getBasePositionAndOrientation(robot)
    cam_target = [base_pos[0], base_pos[1], base_pos[2] + cam_height]

    p.resetDebugVisualizerCamera(
        cameraDistance=cam_dist,
        cameraYaw=cam_yaw,
        cameraPitch=cam_pitch,
        cameraTargetPosition=cam_target
    )

def debug_info(robot):
    """Wyświetla informacje debugowania o aktualnych kątach przegubów."""
    print("\n" + "=" * 50)
    print("AKTUALNE KĄTY PRZEGUBÓW:")

    for i in range(min(20, p.getNumJoints(robot))):
        angle = p.getJointState(robot, i)[0]
        print(f"Joint {i}: {angle:7.2f}°")

def read_target_positions(sliders):
    """Odczytuje docelowe pozycje X i Z ze sliderów."""
    x_target = p.readUserDebugParameter(sliders['x_target'])
    z_target = p.readUserDebugParameter(sliders['z_target'])
    return x_target, z_target

def main():
    """Główna pętla symulacji."""
    robot = initialize_simulation()
    sliders = create_ui_sliders()
    
    while True:
        x_target, z_target = read_target_positions(sliders)
        
        left_targets = solve_ik(x_target, z_target, "left")
        right_targets = solve_ik(x_target, z_target, "right")
        
        joint_targets = combine_leg_targets(left_targets, right_targets)
        apply_joint_targets(robot, joint_targets)
        
        debug_info(robot)
        update_camera(robot, sliders)
        
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
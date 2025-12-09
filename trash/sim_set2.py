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
        'x_target': p.addUserDebugParameter("  X target", -0.1, 0.1, 0.0),
        'y_target': p.addUserDebugParameter("  Y target", -0.1, 0.1, 0.0),
        'z_target': p.addUserDebugParameter("  Z target", 0, 0.302, 0.302)
    }
    return sliders

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
        joint_targets[7] = -hip_roll 
        joint_targets[8] = hip_pitch
        joint_targets[9] = -hip_pitch
        joint_targets[10] = -(knee_pitch + hip_pitch)
        joint_targets[11] = knee_pitch + hip_pitch
        joint_targets[12] = hip_roll
        joint_targets[13] = 0  # fixed joint

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
    y_target = p.readUserDebugParameter(sliders['y_target'])
    z_target = p.readUserDebugParameter(sliders['z_target'])
    return x_target, y_target, z_target

def main():
    """Główna pętla symulacji."""
    robot = initialize_simulation()
    sliders = create_ui_sliders()
    
    while True:
        x_target, y_target, z_target = read_target_positions(sliders)
        
        left_targets = solve_ik_3d(x_target, -y_target, z_target, "left")
        right_targets = solve_ik_3d(x_target, -y_target, z_target, "right")
        
        joint_targets = combine_leg_targets(left_targets, right_targets)
        apply_joint_targets(robot, joint_targets)
        
        debug_info(robot)
        update_camera(robot, sliders)
        
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
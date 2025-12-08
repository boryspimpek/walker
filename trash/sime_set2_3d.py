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
robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=True)
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
z_slider = p.addUserDebugParameter("  Z target (gora-dol)", -0.135, 0.174, 0.067+0.04)

# ========================================
# IK — Funkcja inverse kinematics 3D
# ========================================
def solve_ik_3d(x, y, z, leg, elbow_up=False):
    """
    Rozwiązuje IK dla nogi robota w 3D.
    
    Args:
        x, y, z: współrzędne docelowe stopy
        leg: "left" lub "right"
        elbow_up: kierunek łokcia (nieużywane obecnie)
    
    Returns:
        Lista 14 wartości kątów dla wszystkich jointów
    """
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

# ========================================
# GŁÓWNA PĘTLA
# ========================================
print("Symulacja uruchomiona. Użyj sliderów do kontroli pozycji stóp.")
print("=" * 60)

# Przechowywanie poprzednich wartości dla obu nóg
prev_joint_targets_left = [0.0] * 14
prev_joint_targets_right = [0.0] * 14

while True:
    x_target = p.readUserDebugParameter(x_slider)
    y_target = p.readUserDebugParameter(y_slider)
    z_target = p.readUserDebugParameter(z_slider)
    
    try:
        # Oblicz IK dla lewej nogi
        joint_targets_left = solve_ik_3d(x_target, y_target, z_target, leg="left")
        prev_joint_targets_left = joint_targets_left
        
        # Oblicz IK dla prawej nogi
        joint_targets_right = solve_ik_3d(x_target, y_target, z_target, leg="right")
        prev_joint_targets_right = joint_targets_right
        
    except ValueError as e:
        # Jeśli pozycja poza zasięgiem, użyj poprzednich wartości
        print(f"⚠ BŁĄD IK: {e}")
        joint_targets_left = prev_joint_targets_left
        joint_targets_right = prev_joint_targets_right

    # Sterowanie lewą nogą (jointy 0-6)
    for jid in range(7):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets_left[jid],
            force=500
        )
    
    # Sterowanie prawą nogą (jointy 7-13)
    for jid in range(7, 14):
        p.setJointMotorControl2(
            robot, jid,
            p.POSITION_CONTROL,
            targetPosition=joint_targets_right[jid],
            force=500
        )

    # Aktualizacja kamery
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
    
    print("-" * 60)
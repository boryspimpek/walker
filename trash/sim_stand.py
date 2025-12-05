import pybullet as p
import pybullet_data
import time
import numpy as np


# ========================================
# KONFIGURACJA STAŁYCH
# ========================================
LEFT_EE = 6
RIGHT_EE = 13

# ========================================
# FUNKCJE POMOCNICZE
# ========================================
def init_simulation():
    """Inicjalizacja środowiska + ładowanie robota"""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
    
    return robot


def configure_dynamics(robot):
    """Ustawienia tłumienia i tarcia"""
    num_joints = p.getNumJoints(robot)

    for j in range(num_joints):
        p.changeDynamics(robot, j,
                         linearDamping=0.02,
                         angularDamping=0.02,
                         jointDamping=0.05)

    for ee in [LEFT_EE, RIGHT_EE]:
        p.changeDynamics(robot, ee,
                         lateralFriction=1.3,
                         spinningFriction=0.1,
                         rollingFriction=0.1)


def create_sliders():
    """Tworzy wszystkie slidery i zwraca je w słowniku"""

    sliders = {}

    sliders["cam_dist"] = p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.5)
    sliders["cam_yaw"] = p.addUserDebugParameter("  Obrot kamery (Yaw)", -180, 180, 56)
    sliders["cam_pitch"] = p.addUserDebugParameter("  Nachylenie kamery (Pitch)", -89, 89, 0)
    sliders["cam_h"] = p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0.0)
    return sliders


def get_leg_targets():
    left = [0, 0.04, 0.006]
    right = [0, -0.04, 0.006]
    return left, right


def compute_IK(robot, left_target, right_target):
    """Oblicza IK i scala obie nogi w jedną tablicę jointów
    Ta funkcja jest poprawna i dobrze mapuje kąty"""
    num_joints = p.getNumJoints(robot)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])

    left_ik = p.calculateInverseKinematics(
        robot, LEFT_EE, left_target, target_orientation)

    right_ik = p.calculateInverseKinematics(
        robot, RIGHT_EE, right_target, target_orientation)

    joint_angles = [0] * num_joints

    for i in range(6):  # lewa noga
        joint_angles[i] = left_ik[i]

    for i in range(6, 12):  # prawa noga -> przesunięcie o 1
        joint_angles[i + 1] = right_ik[i]

    return joint_angles


def apply_joint_angles(robot, joint_angles):
    """Ustawiający wszystkie stawy"""
    for i, angle in enumerate(joint_angles):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                targetPosition=angle,             force=2000,
            positionGain=0.8,      
            velocityGain=0.5, 
            maxVelocity=10   
)


def update_camera(robot, sliders):
    """Kontrola kamery sliderami"""
    dist = p.readUserDebugParameter(sliders["cam_dist"])
    yaw = p.readUserDebugParameter(sliders["cam_yaw"])
    pitch = p.readUserDebugParameter(sliders["cam_pitch"])
    height = p.readUserDebugParameter(sliders["cam_h"])

    pos, _ = p.getBasePositionAndOrientation(robot)
    target = [pos[0], pos[1], pos[2] + height]

    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)


# ========================================
# GŁÓWNY PROGRAM
# ========================================
def main():
    robot = init_simulation()
    configure_dynamics(robot)
    sliders = create_sliders()

    while True:
        left_target, right_target = get_leg_targets()
        joint_angles = compute_IK(robot, left_target, right_target)
        left_leg_angles = joint_angles[:7]
        right_leg_angles = joint_angles[7:14]
        print("Lewa noga:  ", ["{:.3f}".format(a) for a in left_leg_angles])
        print("Prawa noga: ", ["{:.3f}".format(a) for a in right_leg_angles])        
        apply_joint_angles(robot, joint_angles)
        update_camera(robot, sliders)


        p.stepSimulation()
        time.sleep(1 / 240.0)


if __name__ == "__main__":
    main()

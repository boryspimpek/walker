import pybullet as p
import pybullet_data
import time

LEFT_EE = 6
RIGHT_EE = 13


def compute_IK(robot):
    # funkcja zwraca kąty w postaci: [4, -0.002, 0.006, 0.003, 0.0, -4.66, 0, -7.62, -0.00, -0.0023, -0.0039, 0.00, 7.841461850645587e-08, 0]
    num_joints = p.getNumJoints(robot)
    left_target = [0.05, 0.104, 0.0]
    right_target = [-0.05, -0.104, 0.0]

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

    left_leg_angles = joint_angles[:7]
    right_leg_angles = joint_angles[7:14]
    print("Lewa noga:  ", ["{:.3f}".format(a) for a in left_leg_angles])
    print("Prawa noga: ", ["{:.3f}".format(a) for a in right_leg_angles])        

    return joint_angles

def show_robot_position(robot, angles, duration=2):
    num_joints = p.getNumJoints(robot)
    
    # Ustaw wszystkie kąty
    for i in range(num_joints):
        p.resetJointState(robot, i, angles[i])  # resetJointState ustawia od razu
    
    # Wyświetl przez określony czas
    start_time = time.time()
    while time.time() - start_time < duration:
        p.stepSimulation()
        time.sleep(1/240)  # konieczne, żeby GUI się odświeżyło

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Załaduj podłoże i robota
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.302], useFixedBase=False)
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.302], [0, 0, 0, 1])

    p.stepSimulation()
    time.sleep(10/240)
    
    num_joints = p.getNumJoints(robot)

    angles1 = ['0.000', '-1.389', '-2.117', '0.001', '-0.729', '-0.000', '0.000', '-0.000', '-1.385', '2.122', '-0.001', '0.738', '0.000', '0.000']
    angles1 = [float(a) for a in angles1]

    angles2 = ['0.000', '-0.409', '-1.212', '0.002', '-0.805', '-0.000', '0.000','-0.000', '-1.087', '1.241', '-0.000', '0.154', '0.000', '0.000']
    angles2 = [float(b) for b in angles2]
            
    while True:
        show_robot_position(robot, angles1, duration=2)  
        p.resetBasePositionAndOrientation(robot, [0, 0, 0.302], [0, 0, 0, 1])
        angles3 = compute_IK(robot)
        show_robot_position(robot, angles3, duration=2)  
        p.resetBasePositionAndOrientation(robot, [0, 0, 0.302], [0, 0, 0, 1])
if __name__ == "__main__":
    main()

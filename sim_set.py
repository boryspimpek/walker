import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.22], useFixedBase=True)

end_effector_index = 2  

# Parametry kamery
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.5)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 0)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -0)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

while True:
    x = 0.056
    y = 0
    z = 0.053
    
    target_position = [x, y, z]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Oblicz kinematykę odwrotną
    joint_positions = p.calculateInverseKinematics(
        robot,
        end_effector_index,
        target_position,
        target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    print("OBLICZONE KĄTY PRZEGUBÓW:")
    for i, angle in enumerate(joint_positions):
        print(f"Przegub {i}: {angle:.4f} rad ({np.degrees(angle):.2f}°)")
    print(f"{'='*50}\n")   

    # Ustaw pozycje przegubów
    for i in range(len(joint_positions)):
        p.setJointMotorControl2(
            robot, 
            i, 
            p.POSITION_CONTROL, 
            targetPosition=joint_positions[i],
            force=2000
        )
    
    # Ustawienia kamery
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    
    distance = p.readUserDebugParameter(camera_distance)
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    height_offset = p.readUserDebugParameter(camera_height_offset)
    
    camera_target = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=camera_target
    )
    
    p.stepSimulation()
    time.sleep(1./240.)
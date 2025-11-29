import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.22], useFixedBase=True)

while True:
    x = 0.05
    y = 0
    z = 0.08
    
    target_position = [x, y, z]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    joint_positions = p.calculateInverseKinematics(
        robot,
        2,
        target_position,
        target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    print("OBLICZONE KĄTY PRZEGUBÓW:")
    for i, angle in enumerate(joint_positions):
        print(f"Przegub {i}: {angle:.4f} rad ({np.degrees(angle):.2f}°)")
    print(f"{'='*50}\n")   

    for i in range(len(joint_positions)):
        p.setJointMotorControl2(
            robot, 
            i, 
            p.POSITION_CONTROL, 
            targetPosition=joint_positions[i],
            force=2000,
            positionGain=0.8,      
            velocityGain=0.5, 
            maxVelocity=10   
        )
    
    # Ustawienia kamery
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    height_offset = 0
    camera_target = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(
        cameraDistance = 0.5,
        cameraYaw = 0,
        cameraPitch = 0,
        cameraTargetPosition=camera_target
    )
    
    p.stepSimulation()
    time.sleep(1./240.)
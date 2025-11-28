import pybullet as p
import pybullet_data
import time
import math
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.22], useFixedBase=True)

end_effector_index = 2  

camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.2)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 15)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -8)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

amplitude = 0.08  
frequency = 0.5   
time_elapsed = 0.0

while True:
    target_z = amplitude + amplitude * math.sin(2 * math.pi * frequency * time_elapsed)
    target_position = [0, 0, target_z]  
    
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    joint_positions = p.calculateInverseKinematics(
        robot,
        end_effector_index,
        target_position,
        target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    
    for i in range(len(joint_positions)):
        p.setJointMotorControl2(
            robot, 
            i, 
            p.POSITION_CONTROL, 
            targetPosition=joint_positions[i],
            force=500
        )
    
    time_elapsed += 1./240.
    
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    
    distance = p.readUserDebugParameter(camera_distance)
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    height_offset = p.readUserDebugParameter(camera_height_offset)
    
    target_position = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=target_position
    )
    
    p.stepSimulation()
    time.sleep(1./240.)
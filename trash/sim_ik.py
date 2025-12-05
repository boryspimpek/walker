import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.325], useFixedBase=True)

end_effector_index = 6

target_x = p.addUserDebugParameter("Pozycja X", -0.5, 0.5, 0.0)
target_y = p.addUserDebugParameter("Pozycja Y", -0.5, 0.5, 0.0)
target_z = p.addUserDebugParameter("Pozycja Z", 0.0, 0.5, 0.0)

camera_distance = p.addUserDebugParameter("Odleglosc", 0.5, 10, 0.5)
camera_yaw = p.addUserDebugParameter("Obrot (yaw)", -180, 180, 0)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -0)
camera_height_offset = p.addUserDebugParameter("Wysokosc kamery", -2, 2, 0.0)

while True:
    x = p.readUserDebugParameter(target_x)
    y = p.readUserDebugParameter(target_y)
    z = p.readUserDebugParameter(target_z)
    
    target_position = [x, y, z]
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
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
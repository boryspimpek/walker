import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.325], useFixedBase=False)

sliders = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    slider = p.addUserDebugParameter(name, -3.14, 3.14, 0)
    sliders.append(slider)

camera_distance = p.addUserDebugParameter("  Odleglosc", 0.5, 10, 0.0)
camera_yaw = p.addUserDebugParameter("  Obrot (yaw)", -180, 180, 15)
camera_pitch = p.addUserDebugParameter("  Nachylenie (pitch)", -89, 89, -10)
camera_height_offset = p.addUserDebugParameter("  Wysokosc kamery", -2, 2, 0.0)

while True:
    for i, slider in enumerate(sliders):
        angle = p.readUserDebugParameter(slider)
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                targetPosition=angle, force=500)
    
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
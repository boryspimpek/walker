import pybullet as p
import pybullet_data
import time
import math


"""
ROBOT STRUCTURE
Base: -1
Link 0: right_hip_1          | Joint: Revolute 85          | Type: REV | Parent: -1     |(roll)
Link 1: right_thigh_1        | Joint: Revolute 86          | Type: REV | Parent: 0      |(pitch)
Link 2: right_knee_1         | Joint: Rigid 87             | Type: FIX | Parent: 1 
Link 3: right_shin_1         | Joint: Revolute 88          | Type: REV | Parent: 2      |(pitch)
Link 4: right_ankle_1        | Joint: Revolute 89          | Type: REV | Parent: 3      |(pitch)
Link 5: right_foot_1         | Joint: Revolute 91          | Type: REV | Parent: 4      |(roll)
Link 6: end_effector_1       | Joint: Rigid 93             | Type: FIX | Parent: 5
"""

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.35], useFixedBase=True)

# Dodaj slidery do ręcznego sterowania
sliders = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    slider = p.addUserDebugParameter(name, -3.14, 3.14, 0)
    sliders.append(slider)

# Parametry kamery
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.25)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 45)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -30)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

# Pętla symulacji
while True:
    # Odczytaj wartości ze sliderów i ustaw joiny
    for i, slider in enumerate(sliders):
        angle = p.readUserDebugParameter(slider)
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                targetPosition=angle, force=500)
    
    # Pobierz pozycję robota
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)
    
    # Odczytaj parametry kamery
    distance = p.readUserDebugParameter(camera_distance)
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    height_offset = p.readUserDebugParameter(camera_height_offset)
    
    # Ustaw kamerę tak, żeby śledziła robota
    target_position = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=target_position
    )
    
    p.stepSimulation()
    time.sleep(1./240.)
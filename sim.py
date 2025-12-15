import pybullet as p
import pybullet_data
import time
import math

from servos import MoveServo

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.325], useFixedBase=True)

sliders = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    slider = p.addUserDebugParameter(name, -3.14, 3.14, 0)
    sliders.append(slider)

camera_distance = p.addUserDebugParameter("  Odleglosc", 0.5, 10, 0.0)
camera_yaw = p.addUserDebugParameter("  Obrot (yaw)", -180, 180, 75)
camera_pitch = p.addUserDebugParameter("  Nachylenie (pitch)", -89, 89, -10)
camera_height_offset = p.addUserDebugParameter("  Wysokosc kamery", -2, 2, 0.0)

servo_angles = [0]*14  # lista dla kątów serw

while True:
    # left leg
    servo_angles[0]  = 90 - math.degrees(p.readUserDebugParameter(sliders[0]))  # servo 1
    servo_angles[1]  = 90 + math.degrees(p.readUserDebugParameter(sliders[1]))  # servo 2
    servo_angles[2]  = 90 + math.degrees(p.readUserDebugParameter(sliders[2]))
    servo_angles[3]  = 90 + math.degrees(p.readUserDebugParameter(sliders[3]))  # servo 3
    servo_angles[4]  = 90 + math.degrees(p.readUserDebugParameter(sliders[4]))  
    servo_angles[5]  = 90 + math.degrees(p.readUserDebugParameter(sliders[5]))  # servo 4
    servo_angles[6]  = 90 + math.degrees(p.readUserDebugParameter(sliders[6]))
    # right leg
    servo_angles[7]  = 90 - math.degrees(p.readUserDebugParameter(sliders[7]))  # servo 5
    servo_angles[8]  = 90 - math.degrees(p.readUserDebugParameter(sliders[8]))  # servo 6
    servo_angles[9]  = 90 + math.degrees(p.readUserDebugParameter(sliders[9]))
    servo_angles[10] = 90 + math.degrees(p.readUserDebugParameter(sliders[10])) # servo 7
    servo_angles[11] = 90 + math.degrees(p.readUserDebugParameter(sliders[11]))
    servo_angles[12] = 90 + math.degrees(p.readUserDebugParameter(sliders[12])) # servo 8
    servo_angles[13] = 90 + math.degrees(p.readUserDebugParameter(sliders[13]))

    MoveServo(1, servo_angles[0])
    MoveServo(2, servo_angles[1])
    MoveServo(3, servo_angles[3])
    MoveServo(4, servo_angles[5])
    MoveServo(5, servo_angles[7])
    MoveServo(6, servo_angles[8])
    MoveServo(7, servo_angles[10])
    MoveServo(8, servo_angles[12])

    print(servo_angles)

    for i, slider in enumerate(sliders):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                targetPosition=p.readUserDebugParameter(slider), 
                                force=500)

    robot_pos, _ = p.getBasePositionAndOrientation(robot)
    distance = p.readUserDebugParameter(camera_distance)
    yaw = p.readUserDebugParameter(camera_yaw)
    pitch = p.readUserDebugParameter(camera_pitch)
    height_offset = p.readUserDebugParameter(camera_height_offset)
    target_position = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=yaw,
                                 cameraPitch=pitch, cameraTargetPosition=target_position)

    p.stepSimulation()
    time.sleep(1./240.)
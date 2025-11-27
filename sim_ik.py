import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Inicjalizacja PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Załaduj robota
robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.22], useFixedBase=True)

# Znajdź końcówkę robota (zakładając, że ostatni joint to końcówka)
num_joints = p.getNumJoints(robot)
end_effector_index = num_joints - 1  # Ostatni joint jako końcówka

# Pobierz informacje o końcówce
end_effector_info = p.getJointInfo(robot, end_effector_index)
print(f"Końcówka: {end_effector_info[1].decode('utf-8')}")

# Parametry kamery (zachowane z oryginalnego kodu)
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.2)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 45)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -8)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

# Parametry ruchu cyklicznego
amplitude = 0.08  # Amplituda ruchu w metrach
frequency = 0.5   # Częstotliwość ruchu w Hz
time_elapsed = 0.0

# Pętla symulacji
while True:
    # Oblicz pozycję docelową końcówki (ruch sinusoidalny w osi Z)
    target_z = 0 + amplitude * math.sin(2 * math.pi * frequency * time_elapsed)
    target_position = [0, 0, target_z]  # X, Y pozostają stałe
    
    # Orientacja docelowa (zachowaj orientację początkową)
    target_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Oblicz odwrotną kinematykę
    joint_positions = p.calculateInverseKinematics(
        robot,
        end_effector_index,
        target_position,
        target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    
    # Ustaw pozycje jointów
    for i in range(len(joint_positions)):
        p.setJointMotorControl2(
            robot, 
            i, 
            p.POSITION_CONTROL, 
            targetPosition=joint_positions[i],
            force=500
        )
    
    # Aktualizuj czas
    time_elapsed += 1./240.
    
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
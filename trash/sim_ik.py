import pybullet as p
import pybullet_data
import time
import numpy as np

# Inicjalizacja PyBullet z GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Załaduj podłoże
planeId = p.loadURDF("plane.urdf")

# Załaduj swojego robota (ZMIEŃ ŚCIEŻKĘ NA SWOJĄ!)
robotId = p.loadURDF("walker.urdf", [0, 0, 0.5], useFixedBase=True)

# Identyfikacja przegubów ruchomych (REV)
num_joints = p.getNumJoints(robotId)
revolute_joints = []

# Parametry kamery
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.25)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 45)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -30)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

print("=== STRUKTURA ROBOTA ===")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]
    
    if joint_type == p.JOINT_REVOLUTE:
        revolute_joints.append(i)
        print(f"Joint {i}: {joint_name} (REVOLUTE)")
    else:
        print(f"Joint {i}: {joint_name} (FIXED)")

# ID końcówki (ostatni link w łańcuchu)
end_effector_index = num_joints - 1

print(f"\nPrzeguby ruchome: {revolute_joints}")
print(f"Końcówka (end-effector): Link {end_effector_index}")

# Pobierz początkową pozycję końcówki
def get_end_effector_pos():
    state = p.getLinkState(robotId, end_effector_index)
    return state[0]  # Pozycja w przestrzeni świata

initial_pos = get_end_effector_pos()
print(f"\nPoczątkowa pozycja końcówki: X={initial_pos[0]:.3f}, Y={initial_pos[1]:.3f}, Z={initial_pos[2]:.3f}")

# Zakresy dla suwaków (dostosuj do swojego robota)
x_range = [initial_pos[0] - 0.3, initial_pos[0] + 0.3]
z_range = [initial_pos[2] - 0.3, initial_pos[2] + 0.3]

# Tworzenie suwaków w GUI
x_slider = p.addUserDebugParameter("Pozycja X", x_range[0], x_range[1], initial_pos[0])
z_slider = p.addUserDebugParameter("Pozycja Z", z_range[0], z_range[1], initial_pos[2])

# Dodatkowe suwaki dla kontroli wizualizacji
show_target = p.addUserDebugParameter("Pokaż cel", 0, 1, 1)

print("\n=== URUCHOMIONO SYMULACJĘ ===")
print("Użyj suwaków 'Pozycja X' i 'Pozycja Z' aby sterować końcówką")

# Główna pętla symulacji
target_sphere = None

try:
    while True:
        # Odczyt wartości z suwaków
        target_x = p.readUserDebugParameter(x_slider)
        target_z = p.readUserDebugParameter(z_slider)
        show_target_sphere = p.readUserDebugParameter(show_target)
        
        # Zachowaj Y na stałym poziomie
        target_pos = [target_x, initial_pos[1], target_z]
        
        # Wizualizacja punktu docelowego
        if show_target_sphere > 0.5:
            if target_sphere is None:
                # Stwórz wizualną sferę w punkcie docelowym
                visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
                target_sphere = p.createMultiBody(baseMass=0, 
                                                 baseVisualShapeIndex=visual_shape,
                                                 basePosition=target_pos)
            else:
                p.resetBasePositionAndOrientation(target_sphere, target_pos, [0, 0, 0, 1])
        elif target_sphere is not None:
            p.removeBody(target_sphere)
            target_sphere = None
        
        # Oblicz kinematykę odwrotną (IK)
        joint_poses = p.calculateInverseKinematics(
            robotId,
            end_effector_index,
            target_pos,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        # Zastosuj obliczone kąty TYLKO do przegubów ruchomych
        for i, joint_id in enumerate(revolute_joints):
            p.setJointMotorControl2(
                robotId,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],  # Użyj indeksu z listy, nie ID przegubu
                force=500
            )

        # Pobierz pozycję robota
        robot_pos, robot_orn = p.getBasePositionAndOrientation(robotId)
        
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

        # Aktualizacja symulacji
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
    print("\n=== ZAMYKANIE SYMULACJI ===")
    p.disconnect()
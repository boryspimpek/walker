import pybullet as p
import pybullet_data
import numpy as np
import time

# Inicjalizacja PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Załaduj płaszczyznę i robota
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("Walker.urdf", [0, 0, 0.5], useFixedBase=True)  # Zmień na swoją ścieżkę

# Parametry kamery
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.25)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 45)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -30)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

# Definicja stawów sterowanych (pomijamy stawy FIX)
controlled_joints = [0, 1, 2]  # Link IDs dla stawów ruchomych

# Znajdź rzeczywiste Joint IDs dla tych linków i ustaw niskie tłumienie
joint_ids = []
for link_id in controlled_joints:
    joint_info = p.getJointInfo(robotId, link_id)
    joint_ids.append(link_id)
    print(f"Link {link_id}: {joint_info[1].decode('utf-8')} | Joint Type: {joint_info[2]}")
    
    # Ustaw niskie wartości tłumienia dla szybszej reakcji
    p.changeDynamics(robotId, link_id, 
                     linearDamping=0.0, 
                     angularDamping=0.0,
                     jointDamping=0.1)

end_effector_link = 3  # Link ID end effectora

# Pobierz początkową pozycję end effectora
initial_ee_state = p.getLinkState(robotId, end_effector_link)
initial_pos = initial_ee_state[0]

# Tworzenie sliderów dla pozycji x, y, z
x_slider = p.addUserDebugParameter("EE Position X", -0.2, 0.2, initial_pos[0])
y_slider = p.addUserDebugParameter("EE Position Y", -0.2, 0.2, initial_pos[1])
z_slider = p.addUserDebugParameter("EE Position Z", 0.275, 0.3, initial_pos[2])

# Orientacja "płaska" względem podłoża
flat_orientation = p.getQuaternionFromEuler([0, 0, 0])  

def calculate_ik(target_pos, target_orientation):
    """
    Oblicza odwrotną kinematykę dla zadanej pozycji i orientacji docelowej
    """
    joint_poses = p.calculateInverseKinematics(
        robotId,
        end_effector_link,
        target_pos,
        targetOrientation=target_orientation,
        maxNumIterations=100,
        residualThreshold=1e-5
    )
    return joint_poses

try:
    while True:
        # Odczytaj wartości ze sliderów
        target_x = p.readUserDebugParameter(x_slider)
        target_y = p.readUserDebugParameter(y_slider)
        target_z = p.readUserDebugParameter(z_slider)
        
        target_pos = [target_x, target_y, target_z]
        
        # Oblicz IK z wymuszeniem płaskiej orientacji
        joint_poses = calculate_ik(target_pos, flat_orientation)
        
        # Ustaw pozycje stawów z wysoką prędkością i tłumieniem prędkości
        for i, joint_id in enumerate(controlled_joints):
            p.setJointMotorControl2(
                robotId,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=2000,
                maxVelocity=100,  # Zwiększona maksymalna prędkość
                positionGain=0.3,  # Gain dla kontrolera pozycji (P)
                velocityGain=1.0   # Gain dla tłumienia prędkości (D)
            )
        
        # Pobierz aktualny stan end effectora
        ee_state = p.getLinkState(robotId, end_effector_link)
        current_pos = ee_state[0]
        current_orn = ee_state[1]

        # Odczytaj parametry kamery
        distance = p.readUserDebugParameter(camera_distance)
        yaw = p.readUserDebugParameter(camera_yaw)
        pitch = p.readUserDebugParameter(camera_pitch)
        height_offset = p.readUserDebugParameter(camera_height_offset)
        robot_pos, robot_orn = p.getBasePositionAndOrientation(robotId)
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
        
except KeyboardInterrupt:
    print("\nZamykanie symulacji...")
    p.disconnect()
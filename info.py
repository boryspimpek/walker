import math
import pybullet as p
import pybullet_data
import time

# Funkcja pomocnicza do konwersji typu jointa na nazwę
def get_joint_type_name(joint_type):
    types = {
        p.JOINT_REVOLUTE: "JOINT_REVOLUTE (obrotowy)",
        p.JOINT_PRISMATIC: "JOINT_PRISMATIC (przesuwny)", 
        p.JOINT_SPHERICAL: "JOINT_SPHERICAL (sferyczny)",
        p.JOINT_PLANAR: "JOINT_PLANAR (płaski)",
        p.JOINT_FIXED: "JOINT_FIXED (stały)"
    }
    return types.get(joint_type, f"Nieznany ({joint_type})")


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", [0, 0, 0.22], useFixedBase=True)
num_joints = p.getNumJoints(robot)

# Parametry kamery (zachowane z oryginalnego kodu)
camera_distance = p.addUserDebugParameter("Odległość", 0.5, 10, 0.1)
camera_yaw = p.addUserDebugParameter("Obrót (yaw)", -180, 180, 15)
camera_pitch = p.addUserDebugParameter("Nachylenie (pitch)", -89, 89, -8)
camera_height_offset = p.addUserDebugParameter("Wysokość kamery", -2, 2, 0.0)

# Poczekaj na stabilizację
for _ in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

base_pos, base_orn = p.getBasePositionAndOrientation(robot)
print(f"\nPozycja bazy robota: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")

print("\n" + "="*80)
print("=== SZCZEGÓŁOWE INFORMACJE O WSZYSTKICH LINKACH ===")
print("="*80)

for i in range(num_joints):
    # Informacje o joint
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    joint_index = joint_info[0]
    joint_lower_limit = joint_info[8]
    joint_upper_limit = joint_info[9]
    
    # Informacje o link
    link_state = p.getLinkState(robot, i)
    link_pos = link_state[0]
    link_orn = link_state[1]  # orientacja (kwaternion)
    link_world_pos = link_state[4]  # pozycja w układzie świata
    link_world_orn = link_state[5]  # orientacja w układzie świata
    
    # Konwersja kwaternionu na Eulera dla lepszej czytelności
    euler_angles = p.getEulerFromQuaternion(link_orn)
    euler_deg = [math.degrees(angle) for angle in euler_angles]
    
    # Stan jointa (aktualny kąt, prędkość, siły)
    joint_state = p.getJointState(robot, i)
    joint_angle = joint_state[0]
    joint_velocity = joint_state[1]
    joint_reaction_forces = joint_state[2]
    joint_motor_torque = joint_state[3]
    
    print(f"\n--- LINK {i}: {joint_name} ---")
    print(f"  Typ jointa: {joint_type} ({get_joint_type_name(joint_type)})")
    print(f"  Indeks: {joint_index}")
    print(f"  Pozycja lokalna: [{link_pos[0]:.3f}, {link_pos[1]:.3f}, {link_pos[2]:.3f}]")
    print(f"  Pozycja światowa: [{link_world_pos[0]:.3f}, {link_world_pos[1]:.3f}, {link_world_pos[2]:.3f}]")
    print(f"  Orientacja (Euler): [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")
    print(f"  Kąt jointa: {math.degrees(joint_angle):.2f}° ({joint_angle:.3f} rad)")
    
    # Sprawdź czy joint ma ograniczenia (dla FIXED joint limits są inne)
    if joint_lower_limit <= joint_upper_limit:  # normalne jointy
        print(f"  Granice ruchu: [{math.degrees(joint_lower_limit):.1f}°, {math.degrees(joint_upper_limit):.1f}°]")
    else:
        print(f"  Granice ruchu: [BRAK - CONTINUOUS JOINT]")
    

print("\n" + "="*80)
print("=== PODSUMOWANIE KĄTÓW JOINTÓW ===")
print("="*80)

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_state = p.getJointState(robot, i)
    joint_angle_deg = math.degrees(joint_state[0])
    
    print(f"Joint {i:2d} ({joint_name:20}): {joint_angle_deg:6.1f}°")

print("\n" + "="*80)
print("=== INFORMACJE O BAZIE ROBOTA ===")
print("="*80)
print(f"Pozycja: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
base_euler = p.getEulerFromQuaternion(base_orn)
base_euler_deg = [math.degrees(angle) for angle in base_euler]
print(f"Orientacja: [{base_euler_deg[0]:.1f}°, {base_euler_deg[1]:.1f}°, {base_euler_deg[2]:.1f}°]")

print("\n" + "="*80)
print("=== STATYSTYKI ===")
print("="*80)
print(f"Łączna liczba jointów: {num_joints}")

# Liczba jointów każdego typu
joint_types_count = {}
joint_categories = {"CONTINUOUS": 0, "LIMITED_REVOLUTE": 0, "FIXED": 0, "OTHER": 0}

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    joint_type = joint_info[2]
    type_name = get_joint_type_name(joint_type)
    
    # Klasyfikacja szczegółowa
    if joint_type == p.JOINT_REVOLUTE:
        if joint_info[8] == 0.0 and joint_info[9] == -1.0:
            joint_categories["CONTINUOUS"] += 1
        else:
            joint_categories["LIMITED_REVOLUTE"] += 1
    elif joint_type == p.JOINT_FIXED:
        joint_categories["FIXED"] += 1
    else:
        joint_categories["OTHER"] += 1
    
    # Klasyfikacja ogólna
    if type_name in joint_types_count:
        joint_types_count[type_name] += 1
    else:
        joint_types_count[type_name] = 1

print("Rozkład typów jointów:")
for joint_type, count in joint_types_count.items():
    print(f"  {joint_type}: {count}")
print("\n" + "="*80)

while True:
    robot_pos, _ = p.getBasePositionAndOrientation(robot)
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
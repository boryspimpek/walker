import numpy as np
import pinocchio as pin
import pybullet as p
import pybullet_data
import time
import os

# Inicjalizacja PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Załaduj model URDF do Pinocchio
model_path = "Walker.urdf"

# SPRAWDŹ CZY PLIK ISTNIEJE
if not os.path.exists(model_path):
    print(f"Error: File {model_path} not found!")
    print(f"Current directory: {os.getcwd()}")
    # List files in current directory
    print("Files in current directory:")
    for file in os.listdir('.'):
        print(f"  {file}")
    exit()

# RÓŻNE SPOSOBY ŁADOWANIA MODELU W ZALEŻNOŚCI OD WERSJI PINOCCHIO
try:
    # Sposób 1: Dla nowszych wersji
    model = pin.buildModelFromUrdf(model_path)
    print("Loaded using buildModelFromUrdf")
except AttributeError:
    try:
        # Sposób 2: Dla starszych wersji
        model = pin.buildModelFromURDF(model_path)
        print("Loaded using buildModelFromURDF")
    except:
        try:
            # Sposób 3: Używając parsera URDF
            from pinocchio.urdf import buildModelFromURDF
            model = buildModelFromURDF(model_path)
            print("Loaded using urdf.buildModelFromURDF")
        except:
            print("Error: Could not load URDF model. Check Pinocchio version.")
            exit()

data = model.createData()

# Reszta kodu pozostaje bez zmian...
q0 = pin.neutral(model)

# Debug information
print("=== Model Information ===")
print(f"Number of joints: {model.njoints}")
print(f"Number of frames: {model.nframes}")

# ... reszta kodu taka sama jak powyżej

print("\n=== Joints ===")
for i, name in enumerate(model.names):
    print(f"Joint {i}: {name}")

print("\n=== Frames ===")
for i, frame in enumerate(model.frames):
    print(f"Frame {i}: {frame.name} (type: {frame.type})")

# Spróbuj znaleźć odpowiednią ramkę dla efektora końcowego
possible_frame_names = ["Component218_1", "component218_1", "link2", "end_effector", "Component218_1_link"]
eef_frame_id = None

for frame_name in possible_frame_names:
    try:
        eef_frame_id = model.getFrameId(frame_name)
        print(f"Found end effector frame: {frame_name} (ID: {eef_frame_id})")
        break
    except Exception as e:
        print(f"Frame {frame_name} not found: {e}")
        continue

if eef_frame_id is None:
    print("Could not find end effector frame. Using last frame.")
    eef_frame_id = model.nframes - 1

# Pożądana pozycja końcowego efektora
target_position = np.array([0.3, 0.0, 0.2])  # Dostosuj do swoich potrzeb

# Opcje IK
max_iter = 1000
eps = 1e-4
dt = 1e-1
damp = 1e-12

def inverse_kinematics(model, data, frame_id, target_pos, q_init, max_iter=1000, eps=1e-4):
    """Oblicza odwrotną kinematykę dla podanej pozycji docelowej"""
    q = q_init.copy()
    
    for i in range(max_iter):
        # Przeprowadź forward kinematics
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        
        # Pobierz aktualną pozycję i jakobian
        frame_placement = data.oMf[frame_id]
        current_pos = frame_placement.translation
        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        
        # Oblicz błąd
        error = target_pos - current_pos
        norm_error = np.linalg.norm(error)
        
        if norm_error < eps:
            print(f"IK converged after {i} iterations")
            break
        
        if i % 100 == 0:
            print(f"Iteration {i}, error: {norm_error:.6f}")
        
        # Oblicz krok przy użyciu pseudoodwrotności z tłumieniem
        J_pinv = np.linalg.solve(J @ J.T + damp * np.eye(3), J).T
        dq = J_pinv @ error
        
        # Zaktualizuj konfigurację
        q = pin.integrate(model, q, dq * dt)
    
    return q, norm_error < eps

# Oblicz IK
print("\n=== Calculating inverse kinematics ===")
q_result, success = inverse_kinematics(model, data, eef_frame_id, target_position, q0, max_iter, eps)

if success:
    print("IK solution found!")
    print(f"Joint positions: {q_result}")
else:
    print("IK failed to converge!")
    q_result = q0  # Użyj konfiguracji domyślnej jeśli IK nie zbiegnie

# Sprawdź końcową pozycję
pin.forwardKinematics(model, data, q_result)
pin.updateFramePlacements(model, data)
final_pos = data.oMf[eef_frame_id].translation
print(f"Target position: {target_position}")
print(f"Final position: {final_pos}")
print(f"Final error: {np.linalg.norm(target_position - final_pos):.6f}")

# Wizualizacja w PyBullet
def visualize_in_pybullet(urdf_path, joint_positions, target_pos):
    """Wizualizuje robota w PyBullet z obliczonymi pozycjami stawów"""
    
    # Załaduj robota do PyBullet
    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(urdf_path, robot_start_pos, robot_start_orientation)
    
    # Ustaw pozycje stawów
    for i in range(len(joint_positions)):
        p.resetJointState(robot_id, i, joint_positions[i])
    
    # Dodaj wizualizację celu
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.5])
    target_id = p.createMultiBody(baseVisualShapeIndex=target_visual, basePosition=target_pos)
    
    # Dodaj linie pokazujące osie współrzędnych
    p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
    
    return robot_id

# Wizualizuj wynik
print("\n=== Visualizing in PyBullet ===")
robot_id = visualize_in_pybullet(model_path, q_result, target_position)

# Główna pętla symulacji
try:
    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)
        
        # Co 100 kroków wyświetl pozycję
        if i % 1000 == 0:
            joint_states = p.getJointStates(robot_id, range(len(q_result)))
            current_positions = [state[0] for state in joint_states]
            print(f"Current joint positions: {current_positions}")
            
except KeyboardInterrupt:
    print("Simulation stopped by user")

p.disconnect()
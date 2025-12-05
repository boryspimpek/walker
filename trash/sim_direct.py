import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Załaduj robota
robot = p.loadURDF("Walker2.urdf", basePosition=[0, 0, 0.35], useFixedBase=True)
p.stepSimulation()

# 1. ZŁAM TO: Ustaw NIEMOŻLIWĄ kombinację kątów
print("=== SPOSÓB 1: Niemożliwe kąty początkowe ===")
# Ustaw złącza tak aby się ze sobą kłóciły
impossible_angles = [1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5, -1.5]

active_idx = 0
for i in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, i)
    if joint_info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        if active_idx < len(impossible_angles):
            p.resetJointState(robot, i, impossible_angles[active_idx])
            active_idx += 1
p.stepSimulation()

# Cel który wymusza zmianę
target = [0.5, 0.3, 0.8]
ik = p.calculateInverseKinematics(
    robot, 6, target,
    p.getQuaternionFromEuler([0.5, 0, 0]),  # Obrócona orientacja!
    maxNumIterations=1000
)

print(f"Cel: {target}")
print("Kąty początkowe (niemożliwe):", [f"{a:.3f}" for a in impossible_angles[:6]])
print("IK wyniki:", [f"{v:.3f}" for v in ik[:6]])

# 2. UŻYJ RÓŻNEGO SOLVERA
print("\n=== SPOSÓB 2: Inny solver ===")
p.resetSimulation()
robot = p.loadURDF("walker2d.urdf", [0, 0, 0.302], useFixedBase=True)
p.stepSimulation()

# Ustaw normalne kąty
for i in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, i)
    if joint_info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        p.resetJointState(robot, i, 0.0)
p.stepSimulation()

# Oblicz IK z JACOBIANEM TRANSAPOSE (solver=0)
ik = p.calculateInverseKinematics(
    robot, 6, [0.5, 0.2, 0.5],
    solver=0,  # JACOBIAN_TRANSPOSE zamiast DLS
    maxNumIterations=1000
)
print("Solver 0 (Jacobian Transpose):", [f"{v:.3f}" for v in ik[:6]])

# 3. NIE PODAWAJ currentPositions - ZMUSZ DO ZERO START
print("\n=== SPOSÓB 3: Bez currentPositions ===")
ik = p.calculateInverseKinematics(
    robot, 6, [0.5, 0.2, 0.5],
    p.getQuaternionFromEuler([0, 0, 0]),
    # NIE podawaj currentPositions!
    maxNumIterations=1000
)
print("Start od zera:", [f"{v:.3f}" for v in ik[:6]])

# 4. UŻYJ RESET POSE jako start
print("\n=== SPOSÓB 4: Użyj restPoses jako start ===")
ik = p.calculateInverseKinematics(
    robot, 6, [0.5, 0.2, 0.5],
    restPoses=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
    maxNumIterations=1000
)
print("Z restPoses:", [f"{v:.3f}" for v in ik[:6]])

# 5. SPRAWDŹ CZY W OGÓLE MOŻLIWY JEST RUCH
print("\n=== SPOSÓB 5: Sprawdź dostępne złącza ===")
for i in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, i)
    if i == 6:  # Lewa stopa
        print(f"Efektor (link {i}): {joint_info[1].decode()}")
    if joint_info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        print(f"Złącze {i}: {joint_info[1].decode()}, zakres: [{joint_info[8]:.2f}, {joint_info[9]:.2f}]")

p.disconnect()
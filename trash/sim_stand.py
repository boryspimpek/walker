import pybullet as p
import pybullet_data
import time

# === Połączenie i świat ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(numSolverIterations=200)

# === Podłoże ===
plane = p.loadURDF("plane.urdf")

# === Załaduj nogę (bez fixed base) ===
startPos = [0, 0, 0.301]  # może trzeba dopasować wysokość
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("Walker2.urdf", startPos, startOrientation, useFixedBase=False)

numJoints = p.getNumJoints(robotId)
foot_link_index = numJoints - 1  # zakładamy że ostatni link = stopa

print("Liczba stawów:", numJoints)
print("Stopa → link index:", foot_link_index)

# === Tłumienie i dynamika stawów ===
for j in range(numJoints):
    p.changeDynamics(robotId, j,
                     linearDamping=0.04,
                     angularDamping=0.04,
                     jointDamping=0.05)

# === Tarcie i kontakt stopy ===
p.changeDynamics(robotId, foot_link_index,
                 lateralFriction=1.3,
                 spinningFriction=0.1,
                 rollingFriction=0.1)

# === Ustaw wszystkie stawy na 0 (pionowo) ===
for j in range(numJoints):
    p.resetJointState(robotId, j, 0)

# === Mała stabilizacja początkowa (krótkie czekanie) ===
for _ in range(300):
    p.stepSimulation()
    time.sleep(1/240)

print("Gotowe — noga powinna stać stabilnie 🙂")

# === Zostaw symulację w pętli ===
while True:
    p.stepSimulation()
    time.sleep(1/240)

import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.2])

# Dodaj slidery do ręcznego sterowania
sliders = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    slider = p.addUserDebugParameter(name, -3.14, 3.14, 0)
    sliders.append(slider)

# Pętla symulacji
while True:
    # Odczytaj wartości ze sliderów i ustaw joiny
    for i, slider in enumerate(sliders):
        angle = p.readUserDebugParameter(slider)
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                targetPosition=angle, force=500)
    
    p.stepSimulation()
    time.sleep(1./240.)
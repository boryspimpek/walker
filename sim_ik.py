import pybullet as p
import pybullet_data
import time
from ik import solve_ik_2d  # twoja funkcja

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", 
                   basePosition=[0, 0, 0.2], useFixedBase=True)

# Interaktywne slidery dla pozycji docelowej
x_slider = p.addUserDebugParameter("Target X", -0.2, 0.2, 0.05)
y_slider = p.addUserDebugParameter("Target Y", -0.2, 0.2, 0.0)
z_slider = p.addUserDebugParameter("Target Z", -0.3, 0.0, -0.15)

prev_pos = None

while True:
    # Odczytaj docelową pozycję
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    
    # Narysuj punkt docelowy
    if prev_pos:
        p.removeUserDebugItem(prev_pos)
    prev_pos = p.addUserDebugPoints([[x, y, z]], [[1, 0, 0]], pointSize=10)
    
    # Oblicz IK
    angles = solve_ik_2d(x, y, z)
    
    if angles:
        # Ustaw kąty w symulacji
        for i, angle in enumerate(angles):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 
                                    targetPosition=angle, force=500)
    
    p.stepSimulation()
    time.sleep(1./240.)
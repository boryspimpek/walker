import mujoco
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("Walkermu.xml")
data = mujoco.MjData(model)

# --- skala modelu ---
scale_factor = 0.01
for i in range(model.ngeom):
    model.geom_size[i] *= scale_factor
    model.geom_pos[i] *= scale_factor

# --- ustaw początkową pozycję wolnej bazy ---
data.qpos[:3] = [0.0, 0.0, 0.05]  # X,Y,Z

# --- ustawienie kamery ---
cam = mujoco.MjvCamera()
cam.lookat[:] = [0.0, 0.0, 0.05]
cam.distance = 0.3
cam.elevation = -30
cam.azimuth = 90

# --- uruchomienie GUI ---
viewer.launch(model, data, cam=cam)

import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", basePosition=[0, 0, 0.0])
end_effector_link = 6  # Link ID end effectora

print("# ROBOT STRUCTURE")
print("# Base: -1")

for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    link_name = info[12].decode('utf-8')
    joint_type = {0: "REV", 1: "PRI", 4: "FIX"}.get(info[2], "OTH")
    parent = info[16]
    
    print(f"# Link {i}: {link_name:20s} | Joint: {joint_name:20s} | Type: {joint_type} | Parent: {parent}")

link_state = p.getLinkState(robot, 6, computeForwardKinematics=True)

print("=== LINK STATE ===")
print(f"1. linkWorldPosition (CoM): {link_state[0]}")
print(f"2. linkWorldOrientation: {link_state[1]}")
print(f"3. localInertialFramePosition: {link_state[2]}")
print(f"4. localInertialFrameOrientation: {link_state[3]}")
print(f"5. worldLinkFramePosition (origin): {link_state[4]}")
print(f"6. worldLinkFrameOrientation: {link_state[5]}")


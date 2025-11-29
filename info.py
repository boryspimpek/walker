import math
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("Walker.urdf", [0, 0, 0.24], useFixedBase=True)
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    p.resetJointState(robot, i, targetValue=0.0, targetVelocity=0.0)


for _ in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

base_pos, base_orn = p.getBasePositionAndOrientation(robot)
print("\n" + "="*80)
print("=== INFORMACJE O BAZIE ROBOTA ===")
print("="*80)
print(f"Pozycja: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
base_euler = p.getEulerFromQuaternion(base_orn)
base_euler_deg = [math.degrees(angle) for angle in base_euler]
print(f"Orientacja: [{base_euler_deg[0]:.1f}°, {base_euler_deg[1]:.1f}°, {base_euler_deg[2]:.1f}°]")

print("\n" + "="*80)
print("=== SZCZEGÓŁOWE INFORMACJE O WSZYSTKICH LINKACH ===")
print("="*80)

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    joint_index = joint_info[0]
    joint_lower_limit = joint_info[8]
    joint_upper_limit = joint_info[9]
    
    link_state = p.getLinkState(robot, i)
    link_pos = link_state[0]
    link_orn = link_state[1]  
    link_world_pos = link_state[4]  
    link_world_orn = link_state[5]  
    
    euler_angles = p.getEulerFromQuaternion(link_orn)
    euler_deg = [math.degrees(angle) for angle in euler_angles]
    
    joint_state = p.getJointState(robot, i)
    joint_angle = joint_state[0]
    
    print(f"\n--- LINK: {i}, name: {joint_name} ---")
    print(f"  Typ jointa: {joint_type}")
    print(f"  Joint Indeks: {joint_index}")
    print(f"  Pozycja lokalna: [{link_pos[0]:.3f}, {link_pos[1]:.3f}, {link_pos[2]:.3f}]")
    print(f"  Pozycja światowa: [{link_world_pos[0]:.3f}, {link_world_pos[1]:.3f}, {link_world_pos[2]:.3f}]")
    print(f"  Orientacja (Euler): [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")
    print(f"  Kąt jointa: {math.degrees(joint_angle):.2f}° ({joint_angle:.3f} rad)")
    
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


while True:
    robot_pos, _ = p.getBasePositionAndOrientation(robot)
    height_offset = 0
    target_position = [robot_pos[0], robot_pos[1], robot_pos[2] + height_offset]
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=0,
        cameraPitch=0,
        cameraTargetPosition=target_position
    )
    p.stepSimulation()
    time.sleep(1./240.)
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_id = p.loadURDF("walker.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robot_id)
print("Liczba jointów:", num_joints)

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_index   = info[0]
    joint_name    = info[1].decode("utf-8")
    joint_type    = info[2]
    parent_index  = info[16]
    link_name     = info[12].decode("utf-8")

    print(f"[{joint_index}]  joint='{joint_name}'   link='{link_name}'   type={joint_type}   parent={parent_index}")

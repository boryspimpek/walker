import pybullet as p
import pybullet_data
import math
import numpy as np
import time

# ========================================
# PARAMETRY CHODU
# ========================================
SWING_WIDTH = 0.08
SWING_HEIGHT = 0.05
SWING_TIME = 0.2
Z_OFFSET = 0.02
X_OFFSET = 0.0

GAIT_SPEED = 0.4  # cykle/s
END_EFFECTOR_INDEX = 6

# Linki powiązane z constraintem gear
JOINT_GEAR_PARENT = 1
JOINT_GEAR_CHILD = 2


def init_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("Walker.urdf", [0, 0, 0.25], useFixedBase=True)

    return robot


# ========================================
# FUNKCJE POMOCNICZE
# ========================================
def trot_gait(phase: float):
    """Zwraca x,z stopy przy zadanej fazie 0 - 1"""
    half_w = SWING_WIDTH / 2

    if phase < SWING_TIME:
        t = phase / SWING_TIME
        angle = math.pi * (1 - t)
        x = -half_w * math.cos(angle) + X_OFFSET
        z = Z_OFFSET + SWING_HEIGHT * math.sin(angle)
    else:
        t = (phase - SWING_TIME) / (1 - SWING_TIME)
        x = -half_w + SWING_WIDTH * t + X_OFFSET
        z = Z_OFFSET

    return x, z


def setup_gear_constraint(robot):
    cid = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=JOINT_GEAR_PARENT,
        childBodyUniqueId=robot,
        childLinkIndex=JOINT_GEAR_CHILD,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    p.changeConstraint(cid, gearRatio=-1, maxForce=100000, erp=1.0)
    return cid


def init_camera_ui():
    return {
        "dist": p.addUserDebugParameter("  Odleglosc kamery", 0.1, 3.0, 0.6),
        "yaw": p.addUserDebugParameter("  Obrot kamery Yaw", -180, 180, 0),
        "pitch": p.addUserDebugParameter("  Nachylenie Pitch", -89, 89, 0),
        "height": p.addUserDebugParameter("  Wysokosc kamery", -1.0, 1.0, 0),
    }


def update_camera(robot, ui):
    robot_pos, _ = p.getBasePositionAndOrientation(robot)

    p.resetDebugVisualizerCamera(
        cameraDistance=p.readUserDebugParameter(ui["dist"]),
        cameraYaw=p.readUserDebugParameter(ui["yaw"]),
        cameraPitch=p.readUserDebugParameter(ui["pitch"]),
        cameraTargetPosition=[
            robot_pos[0],
            robot_pos[1],
            robot_pos[2] + p.readUserDebugParameter(ui["height"])
        ]
    )


def apply_joint_control(robot, target_positions):
    for i in range(min(len(target_positions), p.getNumJoints(robot))):
        if i == JOINT_GEAR_CHILD:
            p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL,
                                    force=0, targetVelocity=0)
        else:
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL,
                                    targetPosition=target_positions[i],
                                    force=2000, positionGain=0.8,
                                    velocityGain=1, maxVelocity=10)


def debug_info(robot):
    print("\n" + "=" * 50)
    print("AKTUALNE KĄTY PRZEGUBÓW:")

    for i in range(min(20, p.getNumJoints(robot))):
        angle = p.getJointState(robot, i)[0]
        print(f"Joint {i}: {np.degrees(angle):7.2f}°")


# ========================================
# GŁÓWNA PĘTLA
# ========================================
def main():
    robot = init_simulation()
    setup_gear_constraint(robot)
    camera_ui = init_camera_ui()

    frame = 0

    while True:
        frame += 1
        phase = (frame * GAIT_SPEED / 240.0) % 1.0

        x, z = trot_gait(phase)

        target_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Neutralna orientacja
        joint_positions = p.calculateInverseKinematics(
            robot,
            END_EFFECTOR_INDEX,
            [x, 0.0, z],
            targetOrientation=target_orientation,
            maxNumIterations=100,
            residualThreshold=1e-4
        )

        apply_joint_control(robot, joint_positions)

        if frame % 60 == 0:
            debug_info(robot)

        update_camera(robot, camera_ui)

        p.stepSimulation()
        time.sleep(1/240)


if __name__ == "__main__":
    main()

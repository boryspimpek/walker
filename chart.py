import time
import math
import matplotlib.pyplot as plt


def solve_ik_2d(x, z, l1, l2, elbow_up=False):
    x_target = x
    z_target = z

    cos_theta2 = (x_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(z_target, x_target) - math.atan2(k2, k1)

    print(f"θ1 = {math.degrees(theta1):.2f}°  θ2 = {math.degrees(theta2):.2f}°")

    return theta1, theta2


def plot_arm_2d(x, z, l1, l2, elbow_up=False):
    theta1, theta2 = solve_ik_2d(x, z, l1, l2, elbow_up)

    # Base
    x0, z0 = 0, 0

    # First joint
    x1 = l1 * math.cos(theta1)
    z1 = l1 * math.sin(theta1)

    # End effector
    x2 = x1 + l2 * math.cos(theta1 + theta2)
    z2 = z1 + l2 * math.sin(theta1 + theta2)

    plt.figure()
    plt.plot([x0, x1, x2], [z0, z1, z2], marker='o', label="Manipulator")
    plt.scatter([x], [z], marker='x', s=80, label="Target")

    plt.axis('equal')
    plt.grid(True)
    plt.title("Prosty manipulator 2D")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    l1 = 98
    l2 = 67
    x = 0
    z = -100

    plot_arm_2d(x, z, l1, l2, elbow_up=False)

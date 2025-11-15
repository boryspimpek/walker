import time
import math
import matplotlib.pyplot as plt


def solve_ik_2d(x, z, l1, l2, elbow_up=False):
    x_target = x + 0
    z_target = z + 0

    # d = math.hypot(x_target, z_target)
    # if d > (l1 + l2) or d < abs(l1 - l2):
    #     raise ValueError("Punkt poza zasięgiem manipulatora")

    cos_theta2 = (x_target**2 + z_target**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))  

    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2  

    k1 = l1 + l2 * math.cos(theta2)
    k2 = l2 * math.sin(theta2)
    theta1 = math.atan2(z_target, x_target) - math.atan2(k2, k1)

    alpha = theta1
    betha = theta1 + theta2
    print(math.degrees(alpha), math.degrees(betha))

    return theta1, theta2, alpha, betha

def plot_arm_2d(x, z, l1, l2, elbow_up=False):
    theta1, theta2, _, _ = solve_ik_2d(x, z, l1, l2, elbow_up)
    x_target = x + 18.16
    z_target = z + 26.25

    x0, z0 = 0, 0
    x1 = l1 * math.cos(theta1)
    z1 = l1 * math.sin(theta1)
    x2 = x1 + l2 * math.cos(theta1 + theta2)
    z2 = z1 + l2 * math.sin(theta1 + theta2)

    x3 = x1 + 31.916 * math.cos(math.radians(55.239))
    z3 = z1 + 31.916 * -math.sin(math.radians(55.239))

    plt.figure(figsize=(6, 6))

    plt.plot([x0, x1, x2], [z0, z1, z2], 'o-', linewidth=3)
    plt.plot([x1, x3], [z1, z3], 'o-', linewidth=3)

    dx = x2 - x1
    dz = z2 - z1
    x4 = x3 + dx
    z4 = z3 + dz
    plt.plot([x3, x4], [z3, z4], 'o-', linewidth=3) 

    plt.plot(x, z, 'rx', markersize=10, label='Cel')
    plt.plot(x_target, z_target, 'rx', markersize=10, label='Cel')
    plt.axis('equal')
    plt.grid(True)
    plt.title("Prosty manipulator 2D")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    l1 = 55
    l2 = 55
    x = 0
    z = -110

    plot_arm_2d(x, z, l1, l2, elbow_up=True)

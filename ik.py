import math

def solve_ik_2d(x_target, z_target, l1, l2, elbow_up=False):
    # d = math.hypot(x_target, z_target)
    # if d > l1 + l2 or d < abs(l1 - l2):
    #     return None

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

    return -theta1, -theta2

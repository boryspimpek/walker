import math

def ik_leg_3dof(x, y, z, L1, L2, clamp_unreachable=True):
    q1 = math.atan2(y, z)  

    z_p = math.hypot(y, z)  

    d = math.hypot(x, z_p)

    num = d*d - L1*L1 - L2*L2
    den = 2.0 * L1 * L2
    if den == 0:
        raise ValueError("Długości L1 i L2 muszą być > 0")
    cos_q3 = num / den

    if abs(cos_q3) > 1.0:
        if clamp_unreachable:
            cos_q3 = max(-1.0, min(1.0, cos_q3))
        else:
            raise ValueError("Punkt poza zasięgiem: cos(q3) = {:.3f}".format(cos_q3))

    sin_q3 = -math.sqrt(max(0.0, 1.0 - cos_q3*cos_q3))
    q3 = math.atan2(sin_q3, cos_q3)

    gamma = math.atan2(z_p, x)  
    beta = math.atan2(L2 * sin_q3, L1 + L2 * cos_q3)
    q2 = gamma - beta

    return q1, q2, q3

def solve_ik_3d(x, y, z, elbow_up=False):
    theta1 = math.atan2(y, z)

    z_prime = math.sqrt(y**2 + z**2)
    x_prime = x

    D = math.sqrt(x_prime**2 + z_prime**2)

    if D > (L1 + L2):
        raise ValueError("Target position outside reachable workspace!")

    theta3 = math.pi - math.acos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))

    alpha = math.atan2(z_prime, x_prime)
    beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    theta2 = alpha - beta
    
    return theta1 , theta2, theta3


# przykładowe długości i punkt
L1 = 0.067   # 20 cm
L2 = 0.067   # 20 cm
x, y, z = 0, 0, 0.067  # przykładowa pozycja stopy (z ujemne = poniżej biodra)

q1, q2, q3 = ik_leg_3dof(x, y, z, L1, L2)
print("q1 (hip roll)  = {:.3f} rad  {:.1f} deg".format(q1, math.degrees(q1)))
print("q2 (hip pitch) = {:.3f} rad  {:.1f} deg".format(q2, math.degrees(q2)))
print("q3 (knee pitch)= {:.3f} rad  {:.1f} deg".format(q3, math.degrees(q3)))

theta1, theta2, theta3 = solve_ik_3d(x, y, z)
print("theta1 (hip roll)  = {:.3f} rad  {:.1f} deg".format(theta1, math.degrees(theta1)))
print("theta2 (hip pitch) = {:.3f} rad  {:.1f} deg".format(theta2, math.degrees(theta2)))
print("theta3 (knee pitch)= {:.3f} rad  {:.1f} deg".format(theta3, math.degrees(theta3)))

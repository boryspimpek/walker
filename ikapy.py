import numpy as np
from ikpy.chain import Chain

# Załaduj obie nogi
left_leg = Chain.from_urdf_file("Walker2.urdf", name="left_leg")
right_leg = Chain.from_urdf_file("Walker2.urdf", name="right_leg")

# Sprawdź strukturę
print("Lewa noga:")
for i, link in enumerate(left_leg.links):
    print(f"  {i}: {link.name}")

print("\nPrawa noga:")
for i, link in enumerate(right_leg.links):
    print(f"  {i}: {link.name}")

# Cele
left_target = [0.0, 0.0, -0.25]
right_target = [0.0, -0.0, -0.25]

# IK
left_solution = left_leg.inverse_kinematics(left_target)
right_solution = right_leg.inverse_kinematics(right_target)

# Weryfikacja
left_fk = left_leg.forward_kinematics(left_solution)
right_fk = right_leg.forward_kinematics(right_solution)

print(f"\nLewa stopa - cel: {left_target}")
print(f"Lewa stopa - osiągnięte: {left_fk[:3, 3]}")
print(f"\nPrawa stopa - cel: {right_target}")
print(f"Prawa stopa - osiągnięte: {right_fk[:3, 3]}")


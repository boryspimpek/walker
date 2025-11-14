import math
import keyboard
import time
import numpy as np

from ik import solve_ik_2d
from servos import prepare_move, move_servo
from config import L1, L2

def control(X=0, Z=-155, STEP=3):
    print("Sterowanie końcówką:")
    print(" W/S = Z+/Z-")
    print(" A/D = X-/X+")
    print(" X = wyjście")
    print()

    while True:
        moved = False

        if keyboard.is_pressed("s"):
            Z += STEP
            moved = True
        if keyboard.is_pressed("w"):
            Z -= STEP
            moved = True
        if keyboard.is_pressed("d"):
            X -= STEP
            moved = True
        if keyboard.is_pressed("a"):
            X += STEP
            moved = True

        if moved:
            ik = solve_ik_2d(X, Z, L1, L2)

            if ik is None:
                print("Poza zasięgiem!", X, Z)
            else:
                t1, t2 = ik
                deg1 = math.degrees(t1)
                deg2 = math.degrees(t2)
                deg3 = deg1 + deg2
                move_servo(2, deg1)
                move_servo(3, deg3)   

                print(f"X={X} Z={Z} | t1={deg1:.1f} t3={deg3:.1f}")

            time.sleep(0.05)

def sweep_x(min_x=-15, max_x=15, step=3, z_fixed=-155, delay=0.1):
    for x in range(min_x, max_x + 1, step):
        ik = solve_ik_2d(x, z_fixed, L1, L2)
        if ik is not None:
            t1, t2 = ik
            deg1 = math.degrees(t1)
            deg2 = math.degrees(t2)
            deg3 = deg1 + deg2
            move_servo(2, deg1)
            move_servo(3, deg3)
            print(f"X={x} Z={z_fixed} | t1={deg1:.1f} t3={deg3:.1f}")
        else:
            print(f"Poza zasięgiem! X={x} Z={z_fixed}")
        time.sleep(delay)

    for x in range(max_x, min_x - 1, -step):
        ik = solve_ik_2d(x, z_fixed, L1, L2)
        if ik is not None:
            t1, t2 = ik
            deg1 = math.degrees(t1)
            deg2 = math.degrees(t2)
            deg3 = deg1 + deg2
            move_servo(2, deg1)
            move_servo(3, deg3)
            print(f"X={x} Z={z_fixed} | t1={deg1:.1f} t3={deg3:.1f}")
        else:
            print(f"Poza zasięgiem! X={x} Z={z_fixed}")
        time.sleep(delay)

def cycle_trajectory(swing_width=30, swing_height=15, x_offset=0, base_z=-120, t_points=15):
    points = []
    half_width = swing_width / 2

    # faza łuku (półokrąg)
    for angle in np.linspace(math.pi, 0, t_points, endpoint=False):
        x = half_width * math.cos(angle) + x_offset
        z = base_z + swing_height * math.sin(angle)
        points.append((x, z))

    # faza liniowa (powrót do startu)
    for t in np.linspace(0, 1, t_points, endpoint=False):
        x = half_width - swing_width * t + x_offset
        z = base_z
        points.append((x, z))

    return points

def run_cycle():
    traj = cycle_trajectory()
    while True:
        for x, z in traj:
            ik = solve_ik_2d(x, z, L1, L2)
            if ik is not None:
                t1, t2 = ik
                deg1 = math.degrees(t1)
                deg2 = math.degrees(t2)
                deg3 = deg1 + deg2
                move_servo(2, deg1)
                move_servo(3, deg3)
            time.sleep(0.05)

if __name__ == "__main__":
    prepare_move()
    run_cycle()

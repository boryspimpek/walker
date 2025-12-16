CYCLE_TIME = 4.0

SWING_TIME = 0.35 # procent of cycle time
SWING_WIDTH = 30
SWING_HEIGHT = 20

X_OFFSET = 0
BASE_Z = -95

# skraca czas w ktorym robot jest sztywno przechylony w bok, zaczyna wcześniej przenosić CoM
HOLD_TIME = SWING_TIME * 1 

FOOT_TILT = 15
HIP_TILT = 5

L1 = 55     
L2 = 55

MAX_SPEED = 2000
ACC = 50

sts_id = [1, 2, 3, 4, 5, 6, 7, 8]   

angle_limits = {1: (75, 105), 2: (30, 150), 3: (50, 180), 4: (75, 105), 5: (75, 105), 6: (30, 150), 7: (0, 130), 8: (75, 105)}

trims = {1: -45, 2: -145, 3: -130, 4: -10, 5: 0, 6: 95, 7: 175, 8: 0}


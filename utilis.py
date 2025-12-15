import time
import math
from st3215 import ST3215
from servos import MoveServo, MoveToPoint

# servo = ST3215('COM3')
sts_id = [1, 2, 3, 4, 5, 6, 7, 8]   


######################################################## List servos
# print(servos := servo.ListServos())

######################################################## Ping servo
# print(alive := servo.PingServo(7))

######################################################## Change ID of servo
# servo.ChangeId(8, 7)
# time.sleep(1)
# print(alive := servo.PingServo(7))

######################################################## Define middle point
# id = 1
# servo.DefineMiddle(id)
# time.sleep(5)
# position = servo.ReadPosition(id)
# angle = servo.servo_to_angle_deg(position)
# print(f"Servo {id} position: {position}, angle: {angle}°")

######################################################## Check positions
# positions = {}
# for id in sts_id:
#     position = servo.ReadPosition(id)
#     print(f"Servo {id} position: {position}")


# for id in sts_id:   
#     MoveServo(id, 90)

MoveServo(2, 90)


# servo.MoveTo(1, 1024, 200, 10, True)
# servo.MoveTo(2, 1024, 200, 10, True)
# servo.MoveTo(3, 1024, 200, 10, True)
# servo.MoveTo(4, 1024, 200, 10, True)
# servo.MoveTo(5, 1024, 200, 10, True)
# servo.MoveTo(6, 1024, 200, 10, True)
# servo.MoveTo(7, 1024, 200, 10, True)
# servo.MoveTo(8, 1024, 200, 10, True)


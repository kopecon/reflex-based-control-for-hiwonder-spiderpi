# This is a dummy version of Board.py

import random


def setBusServoPulse(servo_id, angle, speed):
    for servo in list_of_servos:
        if servo.servo_id == servo_id:
            servo.angle = angle
    return # print(f"Moving joint: {servo_id}, to angle: {angle}, with speed: {speed}")


def getBusServoPulse(servo_id):
    pulse = 0
    for servo in list_of_servos:
        if servo.servo_id == servo_id:
            pulse = servo.angle
    return pulse


def getBusServoVin(servo_id):
    if getBusServoVin.increase is True:
        getBusServoVin.voltage += random.randint(-500, 550)
    else:
        getBusServoVin.voltage -= random.randint(-500, 550)
    if getBusServoVin.voltage <= 9000:
        getBusServoVin.increase = True
    if getBusServoVin.voltage >= 11000:
        getBusServoVin.increase = False
    return getBusServoVin.voltage

getBusServoVin.voltage = 11000
getBusServoVin.increase = False


class Servo:
    def __init__(self, servo_id, angle=500):
        self.servo_id = servo_id
        self.angle = angle


LH_shoulder = Servo(1)
LH_knee = Servo(2)
LH_paw = Servo(3)

LM_shoulder = Servo(4)
LM_knee = Servo(5)
LM_paw = Servo(6)

LF_shoulder = Servo(7)
LF_knee = Servo(8)
LF_paw = Servo(9)

RH_shoulder = Servo(10)
RH_knee = Servo(11)
RH_paw = Servo(12)

RM_shoulder = Servo(13)
RM_knee = Servo(14)
RM_paw = Servo(15)

RF_shoulder = Servo(16)
RF_knee = Servo(17)
RF_paw = Servo(18)

list_of_servos = [
    LH_shoulder,
    LH_knee,
    LH_paw,
    LM_shoulder,
    LM_knee,
    LM_paw,
    LF_shoulder,
    LF_knee,
    LF_paw,
    RH_shoulder,
    RH_knee,
    RH_paw,
    RM_shoulder,
    RM_knee,
    RM_paw,
    RF_shoulder,
    RF_knee,
    RF_paw,
]

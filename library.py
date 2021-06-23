#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

timer = StopWatch()
timer.pause()

lMotor = Motor(Port.B)
rMotor = Motor(Port.C)

# lSensor = ColorSensor(Port.S1)
# rSensor = ColorSensor(Port.S4)

lSensor = Ev3devSensor(Port.S1)
rSensor = Ev3devSensor(Port.S4)

def readPercent(sensor, filter_threshold = 0, maxValue = 255):
    value = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        value = sensor.reflection()
    elif str(type(sensor)) == "<class ''>":
        value = round(sensor.read('RGB')[3] / maxValue * 100)

    return value

def sDev(lSensor = lSensor, filter_threshold = 0, rSensor = rSensor):
    dev = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        dev = lSensor.reflection() - rSensor.reflection()
    elif str(type(lSensor)) == "<class ''>" and str(type(rSensor)) == "<class ''>":
        dev = round((lSensor.read('RGB')[3] - rSensor.read('RGB')[3]) / 255 * 100)

    if filter_threshold != 0:
        return dev
    else:
        return dev // filter_threshold * filter_threshold

def pd(k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    while True:
        dev = sDev()
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev

def pd_1s(sensor, side, gray = 50, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor):
    sign = 0
    if side == 'right':
        sign = -1
    elif side == 'left':
        sign = 1
    dev_old, dev, u = 0, 0, 0
    while True:
        dev = (readPercent(sensor) - gray) * sign
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev

def pdEncoder(angle, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)
    while lMotor.angle() <= angle:
        dev = sDev()
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev
    lMotor.brake()
    rMotor.brake()

def pdEncoderLog(angle, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)

    data_l = []
    data_r = []

    while lMotor.angle() <= angle:
        data_l.append(readPercent(sensor = lSensor))
        data_r.append(readPercent(sensor = rSensor))

        dev = sDev()
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev

    lMotor.brake()
    rMotor.brake()

    log_l = DataLog('lSensor')
    log_r = DataLog('rSensor')

    for i in range (0, len(data_l)):
        log_l.log(data_l[i])
    for i in range (0, len(data_r)):
        log_r.log(data_r[i])

def pdEncoder_1s(angle, sensor, side, gray = 50, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor):
    sign = 1

    if side == 'right':
        sign = -1
    
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)

    while lMotor.angle() <= angle:
        dev = (readPercent(sensor) - gray) * sign
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev

    lMotor.brake()
    rMotor.brake()

def pdCrossings(count = 0, k_p = 1, k_d = 0, vel = 50, minAngle = 99, gray = 35, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)
    
    def body():
        while (not(readPercent(lSensor) <= 16 and readPercent(rSensor) <= 16)) or (lMotor.angle() <= minAngle):
            dev = sDev()
            u = (k_p * dev) + (k_d * (dev - dev_old))
            lMotor.dc(vel + u)
            rMotor.dc(vel - u)
            dev_old = dev

    if count <= 0:
        while True:
            body()
    else:
        for i in range (0, count):
            body()
    lMotor.brake()
    rMotor.brake()

def pdCrossing(k_p = 1, k_d = 0, vel = 50, minAngle = 99, gray = 35, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)
    while (not(readPercent(lSensor) <= 16 and readPercent(rSensor) <= 16)) or (lMotor.angle() <= minAngle):
        dev = sDev()
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev
    lMotor.brake()
    rMotor.brake()

def pidAlignment(kp, kd, ki, sTime, k = 1, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, iSum, time, dev, U = 0, 0, 0, 0, 0
    timer.resume()
    timer.reset()
    while timer.time() <= sTime:
        dev = sDev()
        iSum = iSum + dev
        U = kp * dev + kd * (dev - dev_old) + ki * iSum
        lMotor.dc(U * k)
        rMotor.dc(U * k * -1)
    lMotor.brake()
    rMotor.brake()

def rTurn(kp, kd, ki, time, straight_a, straight_vel, turn_vel, white = 60, black = 17, k = 1, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    lMotor.reset_angle(0)

    while lMotor.angle() <= straight_a:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (readPercent(rSensor) > white):
        lMotor.dc(turn_vel)
        rMotor.dc(-1 * turn_vel)

    while not (readPercent(rSensor) < black):
        lMotor.dc(turn_vel)
        rMotor.dc(-1 * turn_vel)

    lMotor.brake()
    rMotor.brake()
    pidAlignment(kp, kd, ki, time, k, lSensor, rSensor, lMotor, rMotor)

def lTurn(kp, kd, ki, time, straight_a, straight_vel, turn_vel, white = 60, black = 17, k = 1, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    lMotor.reset_angle(0)

    while lMotor.angle() <= straight_a:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (readPercent(lSensor) > white):
        lMotor.dc(-1 * turn_vel)
        rMotor.dc(turn_vel)

    while not (readPercent(lSensor) < black):
        lMotor.dc(-1 * turn_vel)
        rMotor.dc(turn_vel)

    lMotor.brake()
    rMotor.brake()
    pidAlignment(kp, kd, ki, time, k, lSensor, rSensor, lMotor, rMotor)
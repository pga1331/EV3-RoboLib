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

lSensor = ColorSensor(Port.S1)
rSensor = ColorSensor(Port.S4)

# lSensor = Ev3devSensor(Port.S1)
# rSensor = Ev3devSensor(Port.S4)

def readPercent(sensor, filter_threshold = 0, maxValue = 255):
    value = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        value = sensor.reflection()
    elif str(type(sensor)) == "<class ''>":
        value = round(sensor.read('RGB')[3] / maxValue * 100)

    if filter_threshold == 0:
        return value
    else:
        return value // filter_threshold * filter_threshold

def sDev(delta = 0, filter_threshold = 0, lSensor = lSensor, rSensor = rSensor):
    dev = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        dev = lSensor.reflection() - rSensor.reflection()
    elif str(type(lSensor)) == "<class ''>" and str(type(rSensor)) == "<class ''>":
        dev = round((lSensor.read('RGB')[3] - rSensor.read('RGB')[3]) / 255 * 100)

    if filter_threshold == 0:
        return dev + delta
    else:
        return dev // filter_threshold * filter_threshold + delta

def pd(k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    while True:
        dev = sDev(delta = delta, filter_threshold = filter_threshold)
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev

def pd_1s(sensor, side = 'left', gray = 30, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor):
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

def pdEncoder(angle, k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)
    while lMotor.angle() <= angle:
        dev = sDev(delta = delta, filter_threshold = filter_threshold)
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev
    lMotor.brake()
    rMotor.brake()

def pdEncoderLog(angle, k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)

    data_l = []
    data_r = []

    while lMotor.angle() <= angle:
        data_l.append(readPercent(sensor = lSensor))
        data_r.append(readPercent(sensor = rSensor))

        dev = sDev(delta = delta, filter_threshold = filter_threshold)
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

def pdEncoder_1s(angle, sensor, side = 'left', gray = 30, k_p = 1, k_d = 0, vel = 50, lMotor = lMotor, rMotor = rMotor):
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

def pdCrossings(count = 0, k_p = 1, k_d = 0, vel = 50, minAngle = 77, delta = 0, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    def body():
        dev_old, dev, u = 0, 0, 0
        lMotor.reset_angle(0)
        while (not(readPercent(lSensor) <= 16 and readPercent(rSensor) <= 16)) or (lMotor.angle() <= minAngle):
            dev = sDev(delta = delta, filter_threshold = filter_threshold)
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

def pdCrossing(k_p = 1, k_d = 0, vel = 50, minAngle = 77, delta = 0, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, dev, u = 0, 0, 0
    lMotor.reset_angle(0)
    while (not(readPercent(lSensor) <= 16 and readPercent(rSensor) <= 16)) or (lMotor.angle() <= minAngle):
        dev = sDev(delta = delta, filter_threshold = filter_threshold)
        u = (k_p * dev) + (k_d * (dev - dev_old))
        lMotor.dc(vel + u)
        rMotor.dc(vel - u)
        dev_old = dev
    lMotor.brake()
    rMotor.brake()

def pidAlignment(k_p = 1, k_d = 0, k_i = 0, time = 200, k = 1, filter_threshold = 0, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    dev_old, iSum, dev, u = 0, 0, 0, 0
    timer.resume()
    timer.reset()
    while timer.time() <= sTime:
        dev = sDev(delta = 0, filter_threshold = filter_threshold)
        iSum = iSum + dev
        u = k_p * dev + k_d * (dev - dev_old) + k_i * iSum
        lMotor.dc(u * k)
        rMotor.dc(u * k * -1)
    lMotor.brake()
    rMotor.brake()

def lTurn(straight_angle, straight_vel, turn_vel, k_p = 1, k_d = 0, k_i = 0, time = 200, k = 1, filter_threshold = 0, white = 40, black = 18, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    lMotor.reset_angle(0)

    while lMotor.angle() <= straight_angle:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (readPercent(lSensor) >= white):
        lMotor.dc(-1 * turn_vel)
        rMotor.dc(turn_vel)

    while not (readPercent(lSensor) <= black):
        lMotor.dc(-1 * turn_vel)
        rMotor.dc(turn_vel)

    lMotor.brake()
    rMotor.brake()
    pidAlignment(k_p = k_p, k_d = k_d, k_i = k_i, time = time, k = k, filter_threshold = filter_threshold, lSensor = lSensor, rSensor = rSensor, lMotor = lMotor, rMotor = rMotor)

def rTurn(straight_angle, straight_vel, turn_vel, k_p = 1, k_d = 0, k_i = 0, time = 200, k = 1, filter_threshold = 0, white = 40, black = 18, lMotor = lMotor, rMotor = rMotor, lSensor = lSensor, rSensor = rSensor):
    lMotor.reset_angle(0)

    while lMotor.angle() <= straight_angle:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (readPercent(lSensor) >= white):
        lMotor.dc(turn_vel)
        rMotor.dc(-1 * turn_vel)

    while not (readPercent(lSensor) <= black):
        lMotor.dc(turn_vel)
        rMotor.dc(-1 * turn_vel)

    lMotor.brake()
    rMotor.brake()
    pidAlignment(k_p = k_p, k_d = k_d, k_i = k_i, time = time, k = k, filter_threshold = filter_threshold, lSensor = lSensor, rSensor = rSensor, lMotor = lMotor, rMotor = rMotor)
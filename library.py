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

devOld, dev, iSum, u, lValue, rValue = 0, 0, 0, 0, 0, 0

def readPercent(sensor, filterThreshold = 0, maxValue = 255):
    value = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        value = sensor.reflection()
    elif str(type(sensor)) == "<class ''>":
        value = round(sensor.read('RGB')[3] / maxValue * 100)

    if filterThreshold == 0:
        return value
    else:
        return value // filterThreshold * filterThreshold

def resetValues(devOld = devOld, dev = dev, u = u, iSum = iSum, lValue = lValue, rValue = rValue, lMotor = lMotor):
    devOld, iSum, dev, u = 0, 0, 0, 0
    lMotor.reset_angle(0)
    lValue, rValue = readPercent(lSensor), readPercent(rSensor)

def sDev(lValue = readPercent(lSensor), rValue = readPercent(rSensor), delta = 0, filterThreshold = 0):
    dev = lValue - rValue

    if filterThreshold == 0:
        return dev + delta
    else:
        return dev // filterThreshold * filterThreshold + delta

def drive(vel, u):
    lMotor.dc(vel + u)
    rMotor.dc(vel - u)

def pdBody(kP = 1, kD = 0, delta = 0, filterThreshold = 0, lValue = lValue, rValue = rValue, devOld = devOld, dev = dev, u = u):
    dev = sDev(lValue = lValue, rValue = rValue, delta = delta, filterThreshold = filterThreshold)
    u = (kP * dev) + (kD * (dev - devOld))
    devOld = dev

    return u

def pidBody(kP = 1, kD = 0, kI = 0, delta = 0, filterThreshold = 0, devOld = devOld, dev = dev, u = u, iSum = iSum):
    dev = sDev(delta = 0, filterThreshold = filterThreshold)
    iSum = iSum + dev
    u = kP * dev + kD * (dev - devOld) + kI * iSum
    
    return u

def pdInf(kP = 1, kD = 0, vel = 50, delta = 0, filterThreshold = 0):
    resetValues()

    while True:
        u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
        drive(vel, u)

def pd_1s(sensor, side = 'left', gray = 30, kP = 1, kD = 0, vel = 50, filterThreshold = 0):
    sign = 0
    if side == 'right':
        sign = -1
    elif side == 'left':
        sign = 1
    
    resetValues()

    if sensor == lSensor:
        rValue = gray
    elif sensor == rSensor:
        lValue = gray

    while True:
        u = pdBody(kP = kP, kD = kD, delta = 0, filterThreshold = filterThreshold)
        drive(vel, u)

def pdEncoder(angle, kP = 1, kD = 0, vel = 50, delta = 0, filterThreshold = 0):
    resetValues()

    while lMotor.angle() <= angle:
        u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
        drive(vel, u)
    
    lMotor.brake()
    rMotor.brake()

def pdEncoderLog(angle, kP = 1, kD = 0, vel = 50, delta = 0, filterThreshold = 0):
    resetValues()

    data_l = []
    data_r = []

    while lMotor.angle() <= angle:
        data_l.append(readPercent(sensor = lSensor))
        data_r.append(readPercent(sensor = rSensor))

        u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
        drive(vel, u)

    lMotor.brake()
    rMotor.brake()

    log_l = DataLog('lSensor')
    log_r = DataLog('rSensor')

    for i in range (0, len(data_l)):
        log_l.log(data_l[i])
    for i in range (0, len(data_r)):
        log_r.log(data_r[i])

def pdEncoder_1s(angle, sensor, side = 'left', gray = 30, kP = 1, kD = 0, vel = 50):
    sign = 0
    if side == 'right':
        sign = -1
    elif side == 'left':
        sign = 1
    
    resetValues()

    if sensor == lSensor:
        rValue = gray
    elif sensor == rSensor:
        lValue = gray

    while lMotor.angle() <= angle:
        u = pdBody(kP = kP, kD = kD, delta = 0, filterThreshold = filterThreshold)
        drive(vel, u)

    lMotor.brake()
    rMotor.brake()

def pdCrossings(count = 0, kP = 1, kD = 0, vel = 50, minAngle = 77, lineThreshold = 16, delta = 0, filterThreshold = 0):
    def body():
        resetValues()

        while (not(readPercent(lSensor) <= lineThreshold and readPercent(rSensor) <= lineThreshold)) or (lMotor.angle() <= minAngle):
            u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
            drive(vel, u)

    if count <= 0:
        while True:
            body()
    else:
        for i in range (0, count):
            body()

    lMotor.brake()
    rMotor.brake()

def pidAlignment(time = 200, k = 1, kP = 1, kD = 0, kI = 0, filterThreshold = 0):
    resetValues()

    timer.resume()
    timer.reset()

    while timer.time() <= time:
        u = pidBody(kP = kP, kD = kD, kI = kI, filterThreshold = filterThreshold)
        drive(0, u * k)

    lMotor.brake()
    rMotor.brake()

def lTurn(straightAngle = 200, time = 200, k = 1, kP = 1, kD = 0, kI = 0, white = 40, black = 18, straightVel = 77, turnVel = 66, filterThreshold = 0):
    resetValues()

    while lMotor.angle() <= straightAngle:
        drive(straightVel, 0)

    while not (readPercent(lSensor) >= white):
        drive(0, -1 * turnVel)

    while not (readPercent(lSensor) <= black):
        drive(0, -1 * turnVel)

    lMotor.brake()
    rMotor.brake()

    pidAlignment(time = time, k = k, kP = kP, kD = kD, kI = kI, filterThreshold = filterThreshold)

def rTurn(straightAngle = 200, time = 200, k = 1, kP = 1, kD = 0, kI = 0, white = 40, black = 18, straightVel = 77, turnVel = 66, filterThreshold = 0):
    resetValues()

    while lMotor.angle() <= straightAngle:
        drive(straightVel, 0)

    while not (readPercent(rSensor) >= white):
        drive(0, turnVel)

    while not (readPercent(rSensor) <= black):
        drive(0, turnVel)

    lMotor.brake()
    rMotor.brake()

    pidAlignment(time = time, k = k, kP = kP, kD = kD, kI = kI, filterThreshold = filterThreshold)

def pdInfAccel(kP = 1, kD = 0, velStart = 20, velTarget = 88, accelAngle = 200, delta = 0, filterThreshold = 0):
    resetValues()

    vel = velStart
    dVel = velTarget - velStart

    while True:
        if vel < velTarget:
            vel = velStart + (lMotor.angle() / accelAngle) * dVel
        
        u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
        drive(vel, u)

def pdCrossingsAccel(count = 0, kP = 1, kD = 0, velStart = 20, velTarget = 88, accelAngle = 200, minAngle = 77, lineThreshold = 16, delta = 0, filterThreshold = 0):
    def body():
        resetValues()

        vel = velStart
        dVel = velTarget - velStart

        while (not(readPercent(lSensor) <= lineThreshold and readPercent(rSensor) <= lineThreshold)) or (lMotor.angle() <= minAngle):
            if vel < velTarget:
                vel = velStart + (lMotor.angle() / accelAngle) * dVel
            
            u = pdBody(kP = kP, kD = kD, delta = delta, filterThreshold = filterThreshold)
            drive(vel, u)

    if count <= 0:
        while True:
            body()
    else:
        for i in range (0, count):
            body()

    lMotor.brake()
    rMotor.brake()


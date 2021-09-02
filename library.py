#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

lMotor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
rMotor = Motor(Port.C)

lSensor = ColorSensor(Port.S1)
rSensor = ColorSensor(Port.S4)

# lSensor = Ev3devSensor(Port.S1)
# rSensor = Ev3devSensor(Port.S4)

def read_percent(sensor, filter_threshold = 0, max_value = 255):
    value = 0

    if str(type(lSensor)) == "<class 'ColorSensor'>" and str(type(rSensor)) == "<class 'ColorSensor'>":
        value = sensor.reflection()
    elif str(type(sensor)) == "<class ''>":
        value = round(sensor.read('RGB')[3] / max_value * 100)

    if filter_threshold == 0:
        return value
    else:
        return value // filter_threshold * filter_threshold

def s_dev(l_value = read_percent(lSensor), r_value = read_percent(rSensor), delta = 0, filter_threshold = 0):
    dev = l_value - r_value

    if filter_threshold == 0:
        return dev + delta
    else:
        return dev // filter_threshold * filter_threshold + delta

def pd(k_p = 1, k_d = 0, vel = 88, delta = 0, filter_threshold = 0, dev_old = 0, l_value = read_percent(lSensor), r_value = read_percent(rSensor)):
    dev = s_dev(l_value = l_value, r_value = r_value, delta = delta, filter_threshold = filter_threshold)
    u = (k_p * dev) + (k_d * (dev - dev_old))

    lMotor.dc(vel + u)
    rMotor.dc(vel - u)

    return dev

def pid(k_p = 1, k_d = 0, k_i = 0, vel = 88, delta = 0, filter_threshold = 0, dev_old = 0, i_sum = 0):
    dev = s_dev(l_value = read_percent(lSensor), r_value = read_percent(rSensor), delta = delta, filter_threshold = filter_threshold)
    u = (k_p * dev) + (k_d * (dev - dev_old)) + (k_i * i_sum)

    lMotor.dc(vel + u)
    rMotor.dc(vel - u)

    i_sum += dev
    return dev, i_sum

def pd_inf(k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0):
    dev_old = 0

    while True:
        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))

def pd_inf_accel(k_p = 1, k_d = 0, vel_start = 20, vel_target = 88, accel_angle = 200, delta = 0, filter_threshold = 0):
    dev_old = 0

    vel = vel_start
    delta_vel = vel_target - vel_start

    while True:
        if vel < vel_target:
            vel = vel_start + delta_vel * (lMotor.angle() / accel_angle)
        else:
            vel = vel_target

        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))

def pd_encoder(angle, k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0):
    dev_old = 0
    lMotor.reset_angle(0)

    while abs(lMotor.angle()) <= angle:
        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))
    
    lMotor.brake()
    rMotor.brake()

def pd_encoder_1s(angle, sensor = 'left', side = 'left', k_p = 1, k_d = 0, vel = 50, gray = 40, filter_threshold = 0):
    dev_old = 0
    lMotor.reset_angle(0)

    if sensor == 'left':
        if side == 'left':
            while abs(lMotor.angle()) <= angle:
                dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = 0, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = gray)
        elif side == 'right':
            while abs(lMotor.angle()) <= angle:
                dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = 0, filter_threshold = filter_threshold, dev_old = dev_old, l_value = gray, r_value = read_percent(lSensor))
    elif sensor == 'right':
        if side == 'left':
            while abs(lMotor.angle()) <= angle:
                dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = 0, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(rSensor), r_value = gray)
        elif side == 'right':
            while abs(lMotor.angle()) <= angle:
                dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = 0, filter_threshold = filter_threshold, dev_old = dev_old, l_value = gray, r_value = read_percent(rSensor))

    lMotor.brake()
    rMotor.brake()

def pd_crossings(count = 0, k_p = 1, k_d = 0, vel = 50, min_angle = 99, line_threshold = 24, delta = 0, filter_threshold = 0):
    def body():
        dev_old = 0
        lMotor.reset_angle(0)

        while (not(read_percent(lSensor) <= line_threshold and read_percent(rSensor) <= line_threshold)) or (abs(lMotor.angle()) <= min_angle):
            dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))
    
    if count <= 0:
        while True:
            body()
    else:
        for i in range (0, count):
            body()

    lMotor.brake()
    rMotor.brake()

def pd_crossings_accel(count = 0, k_p = 1, k_d = 0, vel_start = 20, vel_target = 88, accel_angle = 200, min_angle = 99, line_threshold = 24, delta = 0, filter_threshold = 0):
    def body():
        dev_old = 0
        lMotor.reset_angle(0)

        vel = vel_start
        delta_vel = vel_target - vel_start

        while (not(read_percent(lSensor) <= line_threshold and read_percent(rSensor) <= line_threshold)) or (abs(lMotor.angle()) <= min_angle):
            if vel < vel_target:
                vel = vel_start + delta_vel * (lMotor.angle() / accel_angle)
            else:
                vel = vel_target

            dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))
    
    if count <= 0:
        while True:
            body()
    else:
        for i in range (0, count):
            body()

    lMotor.brake()
    rMotor.brake()

def pid_alignment(time = 300, k_p = 1.5, k_d = 6, k_i = 0.004):
    dev_old, i_sum = 0, 0

    timer = StopWatch()
    timer.resume()
    timer.reset()

    while timer.time() <= time:
        dev_old, i_sum = pid(k_p = k_p, k_d = k_d, k_i = k_i, vel = 0, delta = 0, filter_threshold = 0, dev_old = dev_old, i_sum = i_sum)

    lMotor.brake()
    rMotor.brake()

def l_turn(straight_angle = 200, time = 300, k_p = 1.5, k_d = 6, k_i = 0.004, white = 55, black = 14, straight_vel = 77, turn_vel = 66):
    dev_old, i_sum = 0, 0
    lMotor.reset_angle(0)

    while abs(lMotor.angle()) <= straight_angle:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (read_percent(lSensor) >= white):
        lMotor.dc(-turn_vel)
        rMotor.dc(turn_vel)

    while not (read_percent(lSensor) <= black):
        lMotor.dc(-turn_vel)
        rMotor.dc(turn_vel)

    lMotor.brake()
    rMotor.brake()

    pid_alignment(time = time, k_p = k_p, k_d = k_d, k_i = k_i)

def r_turn(straight_angle = 200, time = 300, k_p = 1.5, k_d = 6, k_i = 0.004, white = 55, black = 14, straight_vel = 77, turn_vel = 66):
    dev_old, i_sum = 0, 0
    lMotor.reset_angle(0)

    while abs(lMotor.angle()) <= straight_angle:
        lMotor.dc(straight_vel)
        rMotor.dc(straight_vel)

    while not (read_percent(rSensor) >= white):
        lMotor.dc(turn_vel)
        rMotor.dc(-turn_vel)

    while not (read_percent(rSensor) <= black):
        lMotor.dc(turn_vel)
        rMotor.dc(-turn_vel)

    lMotor.brake()
    rMotor.brake()

    pid_alignment(time = time, k_p = k_p, k_d = k_d, k_i = k_i)

def pd_encoder_log(angle, k_p = 1, k_d = 0, vel = 50, delta = 0, filter_threshold = 0):
    dev_old = 0
    lMotor.reset_angle(0)

    data_l = []
    data_r = []

    while lMotor.angle() <= angle:
        data_l.append(readPercent(sensor = lSensor))
        data_r.append(readPercent(sensor = rSensor))

        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, delta = delta, filter_threshold = filter_threshold, dev_old = dev_old, l_value = read_percent(lSensor), r_value = read_percent(rSensor))

    lMotor.brake()
    rMotor.brake()

    log_l = DataLog('lSensor')
    log_r = DataLog('rSensor')

    for i in range (0, len(data_l)):
        log_l.log(data_l[i])
        log_r.log(data_r[i])

# def pd_sync_infinite(vel_b = 50, vel_c = 50, k_p = 1, k_d = 5):
#     dev_old = 0
#     dev = 0
#     u = 0
#     lMotor.reset_angle(0)
#     rMotor.reset_angle(0)

#     while True:
#         dev = rMotor.angle() * abs(vel_b / vel_c) - lMotor.angle()
        
#         u = k_p * dev + k_d * (dev - dev_old)

#         lMotor.dc(vel_b + u)
#         rMotor.dc(vel_c - u)

#         dev_old = dev

# def pd_sync_encoder(vel_b = 50, vel_c = 50, angle = 0, k_p = 1, k_d = 5):
#     dev_old = 0
#     dev = 0
#     u = 0
#     lMotor.reset_angle(0)
#     rMotor.reset_angle(0)

#     while (abs(lMotor.angle()) + abs(rMotor.angle())) / 2 <= abs(angle):
#         dev = rMotor.angle() * abs(vel_b / vel_c) - lMotor.angle()

#         u = k_p * dev + k_d * (dev - dev_old)

#         lMotor.dc(vel_b + u)
#         rMotor.dc(vel_c - u)

#         dev_old = dev
    
#     lMotor.brake()
#     rMotor.brake()

# def pd_sync_arc(vel_b = 50, vel_c = 50, angle = 0, k_p = 1, k_d = 5):
#     dev_old = 0
#     dev = 0
#     u = 0
#     lMotor.reset_angle(0)
#     rMotor.reset_angle(0)

#     sign = abs(vel_b * vel_c - 1) - abs(vel_b * vel_c)

#     if vel_c == 0:
#         ratio = abs(vel_c / vel_b)
#     else:
#         ratio = abs(vel_b / vel_c)

#     while (abs(lMotor.angle()) + abs(rMotor.angle())) / 2 <= abs(angle):
#         dev = rMotor.angle() * ratio + lMotor.angle() * sign

#         u = k_p * dev + k_d * (dev - dev_old)

#         lMotor.dc(vel_b - u * sign)
#         rMotor.dc(vel_c - u)

#         dev_old = dev
    
#     lMotor.brake()
#     rMotor.brake()

def pd_sync_encoder(lVel = 50, rVel = 50, angle = 0, k_p = 1, k_d = 5):
    dev_old = 0
    dev = 0
    u = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    if lVel == 0:
        rMotor.dc(rVel)
        while abs(rMotor.angle()) <= abs(angle):
            pass
        rMotor.brake()
    elif rVel == 0:
        lMotor.dc(lVel)
        while abs(lMotor.angle()) <= abs(angle):
            pass
        lMotor.brake()
    else:
        r_correction = abs(lVel / rVel)

        l_sign = lVel // abs(lVel)
        r_sign = rVel // abs(rVel)

        while (abs(lMotor.angle()) + abs(rMotor.angle())) / 2 <= abs(angle):
            dev = abs(rMotor.angle()) * r_correction - abs(lMotor.angle())
            u = k_p * dev + k_d * (dev - dev_old)
            dev_old = dev

            lMotor.dc(lVel + u * l_sign)
            rMotor.dc(rVel - u * r_sign)
        
        lMotor.brake()
        rMotor.brake()
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.iodevices import Ev3devSensor

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# User class definition

class AdvColorSensor(ColorSensor):
    black_val = 0
    white_val = 100
    def set_map(self, min_val, max_val):
        self.black_val = min_val
        self.white_val = max_val
    def map(self):
        return int((self.reflection() - self.black_val) / (self.white_val - self.black_val) * 100)


class FakeSensor():
    def __init__(self, val):
        self.value = val
    def map(self):
        return self.value


# Create your objects here.
ev3 = EV3Brick()

lMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rMotor = Motor(Port.C)

lSensor = AdvColorSensor(Port.S1)
rSensor = AdvColorSensor(Port.S4)

lCSensor = Ev3devSensor(Port.S2)
rCSensor = Ev3devSensor(Port.S2)

graySensor = FakeSensor(50)

sync_k_p = 1.2
sync_k_d = 1.5

line_k_p = 0.4
line_k_d = 5
line_k_i = 0

line_threshold = 22
ctrl_angle = 60


# Custom functions:

# Base control models
def pd(vel=88, dev_old=0, ls=lSensor, rs=rSensor, k_p=line_k_p, k_d=line_k_d):
    dev = ls.map() - rs.map()
    st = k_p * dev + k_d * (dev - dev_old)

    lMotor.dc(vel + st)
    rMotor.dc(vel - st)

    return dev


def pid(vel=88, dev_old=0, i_sum=0, ls=lSensor, rs=rSensor, k_p=line_k_p, k_d=line_k_d, k_i=line_k_i):
    dev = ls.map() - rs.map()
    st = k_p * dev + k_d * (dev - dev_old) + k_i * i_sum

    lMotor.dc(vel + st)
    rMotor.dc(vel - st)

    return [dev, i_sum + dev]


# Line movement functions
def pd_encoder(vel=88, target_angle=360, k_p=line_k_p, k_d=line_k_d, ls=lSensor, rs=rSensor):
    dev_old = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
        dev_old = pd(vel=vel, dev_old=dev_old, ls=ls, rs=rs, k_p=k_p, k_d=k_d)


def pd_encoder_1s(vel=88, target_angle=360, sensor=lSensor, line_left_side=True, k_p=line_k_p, k_d=line_k_d):
    dev_old = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    if line_left_side:
        while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
            dev_old = pd(vel=vel, dev_old=dev_old, ls=graySensor, rs=sensor, k_p=k_p, k_d=k_d)
    else:
        while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
            dev_old = pd(vel=vel, dev_old=dev_old, ls=sensor, rs=graySensor, k_p=k_p, k_d=k_d)


def pd_crossings(vel=88, target_crossings=1, control_angle=ctrl_angle, threshold=line_threshold, k_p=line_k_p, k_d=line_k_d):
    if target_crossings < 1:
        return 0
    
    dev_old = 0
    
    for i in range(target_crossings):
        lMotor.reset_angle(0)
        while lSensor.map() > threshold or rSensor.map() > threshold or lMotor.angle() < control_angle:
            dev_old = pd(vel=vel, dev_old=dev_old, ls=lSensor, rs=rSensor, k_p=k_p, k_d=k_d)


def pd_crossings_1s(vel=88, target_crossings=1, line_sensor=lSensor, crossing_sensor=rSensor, line_is_left=True, control_angle=ctrl_angle, threshold=line_threshold, k_p=line_k_p, k_d=line_k_d):
    if target_crossings < 1:
        return 0
    
    dev_old = 0
    
    if line_is_left:
        for i in range(target_crossings):
            lMotor.reset_angle(0)
            while crossing_sensor.map() > threshold or lMotor.angle() < control_angle:
                dev_old = pd(vel=vel, dev_old=dev_old, ls=graySensor, rs=line_sensor, k_p=k_p, k_d=k_d)
    else:
        for i in range(target_crossings):
            lMotor.reset_angle(0)
            while crossing_sensor.map() > threshold or lMotor.angle() < control_angle:
                dev_old = pd(vel=vel, dev_old=dev_old, ls=line_sensor, rs=graySensor, k_p=k_p, k_d=k_d)


def pd_encoder_acc(start_vel=33, target_vel=88, acc_angle=200, target_angle=360, ls=lSensor, rs=rSensor, k_p=line_k_p, k_d=line_k_d):
    vel = start_vel
    d_vel = target_vel - start_vel

    dev_old = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
        vel = target_vel if vel >= target_vel else start_vel + d_vel * (lMotor.angle() / acc_angle)
        dev_old = pd(vel=vel, dev_old=dev_old, ls=ls, rs=rs, k_p=k_p, k_d=k_d)


def pd_crossings_acc(start_vel = 33, target_vel = 88, acc_angle = 200, target_crossings = 1, control_angle = ctrl_angle, threshold = line_threshold, ls = lSensor, rs = rSensor, k_p = line_k_p, k_d = line_k_d):
    vel = start_vel
    d_vel = target_vel - start_vel
    
    if target_crossings < 1:
        return 0
    
    dev_old = 0
    
    for i in range(target_crossings):
        lMotor.reset_angle(0)
        while ls.map() > threshold or rs.map() > threshold or lMotor.angle() < control_angle:
            vel = target_vel if vel >= target_vel else start_vel + d_vel * (lMotor.angle() / acc_angle)
            dev_old = pd(vel = vel, dev_old = dev_old, ls = ls, rs = rs, k_p = k_p, k_d = k_d)


def pid_alignment(time=200, k_p=1, k_d=0, k_i=0, ls=lSensor, rs=rSensor):
    dev_old, i_sum = [0] * 2

    timer = StopWatch()
    timer.resume()
    timer.reset()

    while timer.time() < time:
        dev_old, i_sum = pid(vel=0, dev_old=dev_old, i_sum=i_sum, ls=ls, rs=rs, k_p=k_p, k_d=k_d, k_i=k_i)


# Synchronized angle movement functions
def sync_arc_enc(l_vel=88, r_vel=88, angle=360, k_p=sync_k_p, k_d=sync_k_d):
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            rMotor.reset_angle(0)
            while abs(rMotor.angle()) < angle:
                rMotor.dc(r_vel)
    elif r_vel == 0:
        lMotor.reset_angle(0)
        while abs(lMotor.angle()) < angle:
            lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while (abs(lMotor.angle()) + abs(rMotor.angle())) / 2 < angle:
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev


def sync_arc_1s(l_vel=88, r_vel=88, sensor=lSensor, till_less_than=True, control_angle=ctrl_angle, threshold=line_threshold, k_p=sync_k_p, k_d=sync_k_d):
    lMotor.reset_angle(0)
    sgn = 1 if till_less_than else -1
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            while (sgn * sensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
                rMotor.dc(r_vel)
    elif r_vel == 0:
        while (sgn * sensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
                lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while (sgn * sensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev


def sync_arc_2s(l_vel=88, r_vel=88, till_less_than=True, control_angle=ctrl_angle, threshold=line_threshold, k_p=sync_k_p, k_d=sync_k_d):
    lMotor.reset_angle(0)
    sgn = 1 if till_less_than else -1
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            while (sgn * lSensor.map() > sgn * threshold) or (sgn * rSensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
                rMotor.dc(r_vel)
    elif r_vel == 0:
        while (sgn * lSensor.map() > sgn * threshold) or (sgn * rSensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
                lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while (sgn * lSensor.map() > sgn * threshold) or (sgn * rSensor.map() > sgn * threshold) or (abs(lMotor.angle()) < control_angle):
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev


# Color reading functions
def norm_rgb(data = [0] * 3, min = [0] * 3, max = [0] * 3, max_val = 255):
    out = []
    for i in range(0, 3):
        out.append((data[i] - min[i]) / (max[i] - min[i]) * max_val)
    return out


def rgb_to_hsv(data = [0] * 3):
    r, g, b = data

    mx = max(r, g, b)
    mn = min(r, g, b)
    v = mx
    s = 0 if v == 0 else 1 - (mn / mx)

    if mx == mn:
        h = 0
    else:
        if mx == r:
            if g >= b:
                h = 60 * (g - b) / (mx - mn)
            else:
                h = 60 * (g - b) / (mx - mn) + 360
        else:
            if mx == g:
                h = 60 * (b - r) / (mx - mn) + 120
            else:
                h = 60 * (r - g) / (mx - mn) + 240
    
    h, s, v = int(h), float(s), int(v)

    return [h, s, v]


def get_color(hsv = [0] * 3, sum_lim = -5, sat_lim = 0.45, val_whitemin = 112):
    h, s, v = hsv

    if s <= sat_lim:
        if v > val_whitemin:
            return 'WHITE'
        else:
            return 'BLACK'
    else:
        if h <= 11:
            return 'RED'
        if h <= 70:
            return 'YELLOW'
        elif h <= 220:
            return 'GREEN'
        elif h <= 310:
            return 'BLUE'
        else:
            return 'RED'


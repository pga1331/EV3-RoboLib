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

# Create your objects here.
ev3 = EV3Brick()

lMotor = Motor(Port.B, positirve_direction = Direction.COUNTERCLOCKWISE)
rMotor = Motor(Port.C)

lSensor = ColorSensor(Port.S1)
rSensor = ColorSensor(Port.S4)

# lSensor = Ev3devSensor(Port.S1)
# rSensor = Ev3devSensor(Port.S4)

# cSensor = ColorSensor(Port.S2)
# cSensor = Ev3devSensor(Port.S2)

# Functions are defined below:

def ls_map(v_max = 75, v_min = 5):
    return (lSensor.reflection() - v_min) / (v_max - v_min) * 100

def rs_map(v_max = 75, v_min = 5):
    return (rSensor.reflection() - v_min) / (v_max - v_min) * 100

def pd(k_p = 1, k_d = 0, vel = 88, dev_old = 0, l_val = ls_map(), r_val = rs_map()):
    dev = ls_map() - rs_map()
    st = k_p * dev + k_d * (dev - dev_old)

    lMotor.dc(vel + st)
    rMotor.dc(vel - st)

    return dev

def pid(k_p = 1, k_d = 0, k_i = 0, vel = 88, dev_old = 0, i_sum = 0, l_val = ls_map(), r_val = rs_map()):
    dev = ls_map() - rs_map()
    st = k_p * dev + k_d * (dev - dev_old) + k_i * i_sum

    lMotor.dc(vel + st)
    rMotor.dc(vel - st)

    return [dev, i_sum + dev]

def pd_encoder(k_p = 1, k_d = 0, vel = 88, target_angle = 360, l_val = ls_map(), r_val = rs_map()):
    dev_old = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, dev_old = dev_old, l_val = l_val, r_val = r_val)

def pd_crossings(k_p = 1, k_d = 0, vel = 88, target_crossings = 1, control_angle = 90, threshold = 16, l_val = ls_map(), r_val = rs_map()):
    if target_crossings < 1:
        return 0
    
    dev_old = 0
    
    for i in range(target_crossings):
        lMotor.reset_angle(0)
        while ls_map() > threshold or rs_map() > threshold or lMotor.angle() < control_angle:
            dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, dev_old = dev_old, l_val = l_val, r_val = r_val)

def pd_encoder_acc(k_p = 1, k_d = 0, start_vel = 33, target_vel = 88, acc_angle = 200, target_angle = 360, l_val = ls_map(), r_val = rs_map()):
    vel = start_vel
    d_vel = target_vel - start_vel

    dev_old = 0
    lMotor.reset_angle(0)
    rMotor.reset_angle(0)

    while (lMotor.angle() + rMotor.angle()) / 2 < target_angle:
        vel = target_vel if vel >= target_vel else vel_start + d_vel * (lMotor.angle() / start_angle)
        dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, dev_old = dev_old, l_val = l_val, r_val = r_val)

def pd_crossings_acc(k_p = 1, k_d = 0, start_vel = 33, target_vel = 88, acc_angle = 200, target_crossings = 1, control_angle = 90, threshold = 16, l_val = ls_map(), r_val = rs_map()):
    vel = start_vel
    d_vel = target_vel - start_vel
    
    if target_crossings < 1:
        return 0
    
    dev_old = 0
    
    for i in range(target_crossings):
        lMotor.reset_angle(0)
        while ls_map() > threshold or rs_map() > threshold or lMotor.angle() < control_angle:
            vel = target_vel if vel >= target_vel else vel_start + d_vel * (lMotor.angle() / start_angle)
            dev_old = pd(k_p = k_p, k_d = k_d, vel = vel, dev_old = dev_old, l_val = l_val, r_val = r_val)

def pid_alignment(time = 200, k_p = 1, k_d = 0, k_i = 0, l_val = ls_map(), r_val = rs_map()):
    timer = StopWatch()
    timer.resume()
    timer.reset()

    i_sum, dev_old = [0] * 2

    while timer.time() < time:
        dev_old, i_sum = pid(k_p = k_p, k_d = k_d, k_i = k_i, vel = 0, dev_old = dev_old, i_sum = i_sum, l_val = l_val, r_val = r_val)

def sync_arc_enc(l_vel = 88, r_vel = 88, angle = 360, k_p = 1, k_d = 0):
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
    
def sync_arc_ls(l_vel = 88, r_vel = 88, threshold = 20, till_less_than = True, k_p = 1, k_d = 0):
    sgn = 1 if till_less_than else -1
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            while sgn * ls_map() > sgn * threshold:
                rMotor.dc(r_vel)
    elif r_vel == 0:
        while sgn * ls_map() > sgn * threshold:
                lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while sgn * ls_map() > sgn * threshold:
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev

def sync_arc_rs(l_vel = 88, r_vel = 88, threshold = 20, till_less_than = True, k_p = 1, k_d = 0):
    sgn = 1 if till_less_than else -1
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            while sgn * rs_map() > sgn * threshold:
                rMotor.dc(r_vel)
    elif r_vel == 0:
        while sgn * rs_map() > sgn * threshold:
                lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while sgn * rs_map() > sgn * threshold:
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev

def sync_arc_2s(l_vel = 88, r_vel = 88, threshold = 20, till_less_than = True, k_p = 1, k_d = 0):
    sgn = 1 if till_less_than else -1
    if l_vel == 0:
        if r_vel == 0:
            pass
        else:
            while (sgn * ls_map() > sgn * threshold) and (sgn * rs_map() > sgn * threshold):
                rMotor.dc(r_vel)
    elif r_vel == 0:
        while (sgn * ls_map() > sgn * threshold) and (sgn * rs_map() > sgn * threshold):
                lMotor.dc(l_vel)
    else:
        ratio = r_vel / l_vel
        k = abs(l_vel // abs(l_vel) - r_vel // abs(r_vel)) - 1
        lMotor.reset_angle(0)
        rMotor.reset_angle(0)
        dev_old = 0
        while (sgn * ls_map() > sgn * threshold) and (sgn * rs_map() > sgn * threshold):
            dev = rMotor.angle() - lMotor.angle() * ratio
            st = k_p * dev + k_d * (dev - dev_old)

            lMotor.dc(l_vel - k * st)
            rMotor.dc(r_vel - st)
            dev_old = dev

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
        # if h <= 11:
        #     return 'RED'
        if h <= 70:
            return 'YELLOW'
        elif h <= 220:
            return 'GREEN'
        elif h <= 310:
            return 'BLUE'
        else:
            return 'RED'


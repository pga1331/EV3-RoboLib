#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

# Custom libs
from library import *

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

lMotor = Motor(Port.B)
rMotor = Motor(Port.C)

lSensor = ColorSensor(Port.S1)
rSensor = ColorSensor(Port.S4)

# lSensor = Ev3devSensor(Port.S1)
# rSensor = Ev3devSensor(Port.S4)

# timer = StopWatch()
# timer.pause()

# Initialize your constants/variables here

devOld, dev, iSum, u, lValue, rValue = 0, 0, 0, 0, 0, 0

# Write your program here.


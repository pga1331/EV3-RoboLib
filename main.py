#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.iodevices import Ev3devSensor
from library import *
# Library should be imported after device definition

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

lMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rMotor = Motor(Port.C)

lSensor = AdvColorSensor(Port.S1)
rSensor = AdvColorSensor(Port.S4)

lCSensor = Ev3devSensor(Port.S2)
rCSensor = Ev3devSensor(Port.S3)
 
ctrlMotor = Motor(Port.A)
grabMotor = Motor(Port.D)

graySensor = FakeSensor(50)

lSensor.set_map(10, 70)
rSensor.set_map(10, 70)


# adjustment
TURN90_2W = 150
TURN90_1W = 300

sync_k_p = 1.2
sync_k_d = 1.5

line_k_p = 0.36
line_k_d = 2
line_k_i = 0

sound_duration = 150
delay_time = 150

# from library import *


# init
ev3.speaker.set_volume(10, which='Beep')


# Write your program here.

# dLog = DataLog('ls_readings', timestamp=False)

# for i in range(6):
#     dLog.log(lSensor.reflection())
#     ev3.speaker.beep(duration=200)
#     wait(800)

#!/usr/bin/env pybricks-micropython
import pybricks.ev3brick as brick

from  pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor
     ,GyroSensor)
from  pybricks.parameters import (Port,Stop,Direction,Button,Color,SoundFile,ImageFile,Align)

from pybricks.tools import(print,wait,StopWatch)
from  pybricks.robotics import DriveBase
from  pybricks.ev3devio import Ev3devSensor

import threading
import time
from math import cos
from math import atan
from math import sin

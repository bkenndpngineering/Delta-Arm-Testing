from pidev.stepper import stepper
from Slush.Devices import L6470Registers

s0 = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20,
                     steps_per_unit=200, speed=8)

import spidev
import os
from time import sleep
import RPi.GPIO as GPIO
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus

spi = spidev.SpiDev()
cyprus.initialize()
version = cyprus.read_firmware_version()
print(version)

while True:
    if (cyprus.read_gpio() & 0b0001):    # binary bitwise AND of the value returned from read.gpio()
        print("GPIO on port P6 is HIGH")
        sleep(.1)
    else:
        print("GPIO on port P6 is LOW")
        sleep(.1)

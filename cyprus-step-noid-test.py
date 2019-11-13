import os
import spidev
from time import sleep
import RPi.GPIO as GPIO
from pidev.stepper import stepper
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus
from Slush.Devices import L6470Registers
spi = spidev.SpiDev()

cyprus.initialize()
s0 = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20, steps_per_unit=200, speed=8)

def rotate_peng(degrees):
    rotation = degrees * 6400/360
    print(str(rotation))
    cyprus.set_pwm_values(1, period_value=100000, compare_value=50000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
    s0.goTo(int(rotation))
    sleep(1)
    position = s0.get_position_in_units()
    print(str(position))
    cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
    sleep(1)
    s0.set_as_home()

def solenoid_pulse(time):
    cyprus.set_pwm_values(1, period_value=100000, compare_value=50000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
    sleep(time)
    cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)

def motor_rotate(degrees):
    rotation = degrees * 6400/360
    print(str(degrees))
    s0.goTo(int(rotation))
    sleep(1)
    position = s0.get_position_in_units()
    print(str(position))
    s0.set_as_home()

#sleep(1)
#solenoid_pulse(1)
sleep(1)
motor_rotate(180)
sleep(1)

s0.free_all()
spi.close()
GPIO.cleanup()
cyprus.close()

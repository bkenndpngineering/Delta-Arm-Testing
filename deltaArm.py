import os
import spidev
from pidev.stepper import stepper
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus
from Slush.Devices import L6470Registers
import odrive
from RPi_ODrive import ODrive_Ease_Lib
import RPi.GPIO as GPIO
from kinematicFunctions import *
import time

class DeltaArm():
    def __init__(self):
        self.initialized = False    # if the arm is not initialized no motor related commands will work. represents the ODrive harware
        
        self.spi = None             # spidev
        self.stepper = None         # stepper motor object

        self.limitSwitchPin3 = 13   # motor 3 limit switch pin
        self.limitSwitchPin2 = 16   # motor 2 limit switch pin
        self.limitSwitchPin1 = 20   # motor 1 limit switch pin

        self.ODriveSerialNumber1 = 59877000491063   # first ODrive serial number. Controls motors 1 and 2
        self.ODriveSerialNumber2 = 35623325151307   # second ODrive serial number. Controls motor 3

        self.od1 = None             # first ODrive instance
        self.od2 = None             # second ODrive instance

        self.ax0 = None             # axis 0 of the first ODrive. Motor 1
        self.ax1 = None             # axis 1 of the first ODrive. Motor 2
        self.ax2 = None             # axis 0 of the second ODrive. Motor 3

    def rotateStepper(self, degree):
        if self.initialized:
            steps = degree * 6400/360
            self.stepper.goTo(int(steps))
            position = self.stepper.get_position_in_units()
            self.stepper.set_as_home()

    def powerSolenoid(self, state):
        if self.initialized:
            if state == True:
                cyprus.set_pwm_values(1, period_value=100000, compare_value=500000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            elif state == False:
                cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            else:
                return

    def getLim1(self):
        if (cyprus.read_gpio() & 0b0001):
            return False
        else:
            return True

    def getLim2(self):
        if (cyprus.read_gpio() & 0b0010):
            return False
        else:
            return True

    def getLim3(self):
        if (cyprus.read_gpio() & 0b0100):
            return False
        else:
            return True

    def connectODrive(self):
        # find two ODrives and assign them to the correct motors
        # returns false if there is not two ODrives connected or their serial numbers do not match those defined above
        o = ODrive_Ease_Lib.find_ODrives()
        if len(o) != 2:
            return False
        
        if o[0].serial_number == self.ODriveSerialNumber1: od1 = o[0]
        elif o[0].serial_number == self.ODriveSeriaNumber2: od2 = o[0]
        else: return False

        if o[1].serial_number == self.ODriveSerialNumber1: od1 = o[1]
        elif o[1].serial_number == self.ODriveSerialNumber2: od2 = o[1]
        else: return False

        # preserve ODrive object for shutdown method
        self.od1 = od1
        self.od2 = od2

        # initialize ODrive motor axis
        self.ax0 = ODrive_Ease_Lib.ODrive_Axis(od1.axis0)
        self.ax1 = ODrive_Ease_Lib.ODrive_Axis(od1.axis1)
        self.ax2 = ODrive_Ease_Lib.ODrive_Axis(od2.axis0)

        return True

    def homeMotors(self):
        # move the motors to index position. Requirement for position control
        self.ax0.index_and_hold(-1, 1)
        time.sleep(1)
        self.ax1.index_and_hold(-1, 1)
        time.sleep(1)
        self.ax2.index_and_hold(-1, 1)
        time.sleep(1)

        # home motor 1
        self.ax0.set_vel(-20)
        while (not self.getLim1()):
            continue
        self.ax0.set_vel(0)
        self.ax0.set_home()
        time.sleep(1)

        # home motor 2
        self.ax1.set_vel(-20)
        while (not self.getLim2()):
            continue
        self.ax1.set_vel(0)
        self.ax1.set_home()
        time.sleep(1)

        # home motor 3
        self.ax2.set_vel(-20)
        while (not self.getLim3()):
            continue
        self.ax2.set_vel(0)
        self.ax2.set_home()
        time.sleep(1)

        if self.ax0.axis.error != 0: return False
        if self.ax0.axis.motor.error != 0: return False
        if self.ax0.axis.encoder.error != 0: return False
        if self.ax0.axis.controller.error != 0: return False

        if self.ax1.axis.error != 0: return False
        if self.ax1.axis.motor.error != 0: return False
        if self.ax1.axis.encoder.error != 0: return False
        if self.ax1.axis.controller.error != 0: return False
        
        if self.ax2.axis.error != 0: return False
        if self.ax2.axis.motor.error != 0: return False
        if self.ax2.axis.encoder.error != 0: return False
        if self.ax2.axis.controller.error != 0: return False

        return True

    def initialize(self):
        # setup steppermotor, limit switches, and solenoid
        self.spi = spidev.SpiDev()
        cyprus.initialize()
        version = cyprus.read_firmware_version()
        self.stepper = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20, steps_per_unit=200, speed=8)
        self.stepper.home(1)

        # connect to ODrives
        ODriveConnected = self.connectODrive()

        if (ODriveConnected == True):
            # if GPIO and ODrive are setup properly, attempt to home
            HomedMotors = self.homeMotors()
            if (HomedMotors == True): self.initialized = True

    def moveToCoordinates(self, x, y, z):
        if self.initialized:
            (angle1, angle2, angle3) = compute_triple_inverse_kinematics(x, y, z)
            pos1 = angle1 * DEG_TO_CPR
            pos2 = angle2 * DEG_TO_CPR
            pos3 = angle3 * DEG_TO_CPR
            self.ax0.set_pos(pos1)
            self.ax1.set_pos(pos2)
            self.ax2.set_pos(pos3)

    def getCoordinates(self):
        if self.initialized:
            pos1 = self.ax0.get_pos()
            angle1 = pos1 * CPR_TO_DEG

            pos2 = self.ax1.get_pos()
            angle2 = pos2 * CPR_TO_DEG

            pos3 = self.ax2.get_pos()
            angle3 = pos3 * CPR_TO_DEG

            (x, y, z) = forward_kinematics(angle1, angle2, angle3)
            return ((x, y, z))

    def shutdown(self):
        if self.initialized:
            # reset and disconnect from the first ODrive
            try:
                self.od1.reboot()
            except:
                pass

            # reset and disconnect from the second ODrive
            try:
                self.od2.reboot()
            except:
                pass

            # close out of Cyprus and Slush Engine
            self.stepper.free_all()
            self.spi.close()
            GPIO.cleanup()
            cyprus.close()

            self.initialized = False

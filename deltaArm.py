import odrive
from RPi_ODrive import ODrive_Ease_Lib
import RPi.GPIO as GPIO
from kinematicFunctions import *

import time

class DeltaArm():
    self.initialized = False    # if the arm is not initialized no commands will work

    self.limitSwitchPin3 = 13   # motor 3 limit switch pin
    self.limitSwitchPin2 = 16   # motor 2 limit switch pin
    self.limitSwitchPin1 = 20   # motor 1 limit switch pin

    self.ODriveSerialNumber1 = 59877000491063   # first ODrive serial number. Controls motors 1 and 2
    self.ODriveSerialNumber2 = 35623325151307   # second ODrive serial number. Controls motor 3

    self.ax0 = None             # axis 0 of the first ODrive. Motor 1
    self.ax1 = None             # axis 1 of the first ODrive. Motor 2
    self.ax2 = None             # axis 0 of the second ODrive. Motor 3

    def configureGPIO(self):
        # configure limit switches on RPi's GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.limitSwitchPin3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.limitSwitchPin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.limitSwitchPin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        return True

    def getLim(self, pin):
        # poll status of RPi GPIO input pin
        status1 = None
        status2 = None

        # simple debouncing. 100ms dead time
        if (not GPIO.input(pin)): status1 = True
        else: status1 = False
        time.sleep(0.1)
        if (not GPIO.input(pin)): status2 = True
        else: status2 = False

        if status1 and status2: return True
        else: return False

    def getLim3(self):
        # return status of limit switch 3
        if self.initialized:
            return self.getLim(self.limitSwitchPin3)
    def getLim2(self):
        # return status of limit switch 2
        if self.initialized:
            return self.getLim(self.limitSwitchPin2)
    def getLim1(self):
        # return status of limit switch 1
        if self.initialized:
            return self.getLim(self.limitSwitchPin1)

    def connectODrive(self):
        # find two ODrives and assign them to the correct motors
        # returns false if there is not two ODrives connected or their serial numbers do not match those defined above
        o = ODrive_Ease_Lib.find_ODrives()
        if len(o) != 2:
            return False
        
        if o[0].serial_number = self.ODriveSerialNumber1: od1 = o[0]
        elif o[0].serial_number = self.ODriveSeriaNumber2: od2 = o[0]
        else: return False

        if o[1].serial_number = self.ODriveSerialNumber1: od1 = o[1]
        elif o[1].serial_number = self.ODriveSerialNumber2: od2 = o[1]
        else: return False

        # preserve ODrive object for shutdown method
        self.od1 = od1
        self.od2 = od2

        # initialize ODrive motor axis
        self.ax0 = ODrive_Ease_Lib.ODrive_Axis(od1.axis0)
        self.ax1 = ODrive_Ease_Lib.ODrive_Axis(od1.axis1)
        self.ax2 = ODrive_Ease_Lib.ODrive_Axis(od2.axis2)

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
        while (not self.getLim3()))
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
        # configure GPIO for limit switches
        a = self.configureGPIO()
        # connect to ODrives
        b = self.connectODrive()

        if (a == True) and (b == True):
            c = self.homeMotors()
            if (c == True): self.initialized = True

    def moveToCoordinates(self, x, y, z):
        if self.initialized:
            (angle1, angle2, angle3) = compute_triple_inverse_kinematics(x, y, z)
            pos1 = angle1 * DEG_TO_CPR
            pos2 = angle2 * DEG_TO_CPR
            pos3 = angle3 * DEG_TO_CPR
            self.ax0.set_pos(pos1)
            self.ax1.set_pos(pos2)
            self.ax2.set_pos(pos3)

    def getCoordinates(self)
        if self.initialized:
            pos1 = ax0.get_pos()
            angle1 = pos1 * CPR_TO_DEG

            pos2 = ax1.get_pos()
            angle2 = pos2 * CPR_TO_DEG

            pos3 = ax2.get_pos()
            angle3 = pos3 * CPR_TO_DEG

            (x, y, z) = forward_kinematics(angle1, angle2, angle3)
            return ((x, y, z))

    def shutdown(self):
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

        self.initialized = False

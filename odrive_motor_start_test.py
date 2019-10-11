import odrive
from RPi_ODrive import ODrive_Ease_Lib
import odrive.enums
import time
import RPi.GPIO as GPIO

### Setup Limit switches through RPi.GPIO ###

limit_switch1 = 13 # RPi GPIO Pin number
limit_switch2 = 16
limit_switch3 = 20

GPIO.setmode(GPIO.BCM)

# configure switches as input.
# switches are wired to ground, so an internal pull-up resistor is required
GPIO.setup(limit_switch1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit_switch2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit_switch3, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# read by: GPIO.input(pin) --> false if pushed, true if not pushed

### Finish Setup for Limit Switches ###



# make sure to reboot odrive between script runs

# motor 3 ODrive serial number --> 61951538836535
# (swapped ODrives. new motor 3 serial Number --> 35623325151307
# motor 1/2 ODrive serial number --> 59877000491063

# For two ODrives, identify by serial number
o = ODrive_Ease_Lib.find_ODrives()
if len(o) != 2:
    print("Did not find two ODrives")
    exit()
else:
    print("Found two ODrives")

ODrive1_Serial_Number = 59877000491063
#ODrive2_Serial_Number = 61951538836535
ODrive2_Serial_Number = 35623325151307

print("Serial Numbers: ")
print (o[0].serial_number)
print (o[1].serial_number)

# match ODrive to its serial number
if o[0].serial_number == ODrive1_Serial_Number: od1 = o[0]
elif o[0].serial_number == ODrive2_Serial_Number: od2 = o[0]
else:
    print("ODrive serial numbers do not match")
    exit()
if o[1].serial_number == ODrive1_Serial_Number: od1 = o[1]
elif o[1].serial_number == ODrive2_Serial_Number: od2 = o[1]
else:
    print("ODrive serial numbers do not match")
    exit()
print("Connected to ODrives")

# First ODrive controller
# od1.axis0 --> motor 1
ax0 = ODrive_Ease_Lib.ODrive_Axis(od1.axis0)
ax0.index_and_hold(-1, 1)

# od.axis1 --> motor 2
ax1 = ODrive_Ease_Lib.ODrive_Axis(od1.axis1)
ax1.index_and_hold(-1, 1)

# Second ODrive controller
# od.axis0 --> motor 3
ax2 = ODrive_Ease_Lib.ODrive_Axis(od2.axis0)
ax2.index_and_hold(-1, 1)

#configured ODrive limit switches in the following convention: axis 0 --> gpio 2, axis 1 -- > gpio 8
# ax.axis.max_endstop.endstop_state_ --> bool

#create homing sequence with use of endstops
#using RPi.GPIO over ODrive endstops


#delay shutdown for a minute
print("waiting 60 seconds")
time.sleep(60)
print("shutting down")
#reboot both ODrives and exit
try:
    od1.reboot()
except:
    pass
try:
    od2.reboot()
except:
    pass

exit()

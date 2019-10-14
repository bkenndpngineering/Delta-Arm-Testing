import odrive
from RPi_ODrive import ODrive_Ease_Lib
import odrive.enums
import time
import RPi.GPIO as GPIO



### Setup Limit switches through RPi.GPIO ###

#RPi GPIO pin number
limit_switch3 = 13 # motor 3 limit switch
limit_switch2 = 16 # motor 2 lim switch
limit_switch1 = 20 # motor 1 lim switch

GPIO.setmode(GPIO.BCM)

# configure switches as input.
# switches are wired to ground, so an internal pull-up resistor is required
GPIO.setup(limit_switch1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit_switch2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit_switch3, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def getLim(pin):
    status1 = None
    status2 = None
    if (not GPIO.input(pin)): status1 = True
    else: status1 = False
    time.sleep(0.1) # simple debouncing, 100 ms dead time
    if (not GPIO.input(pin)): status2 = True
    else: status2 = False

    if status1 and status2: return True
    else: return False
def getLim1():
    return getLim(limit_switch1)
def getLim2():
    return getLim(limit_switch2)
def getLim3():
    return getLim(limit_switch3)

# read by: GPIO.input(pin) --> false if pushed, true if not pushed
# use: if (not GPIO.input(pin)): print("pushed") --> get if pushed

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
time.sleep(3)
# od.axis1 --> motor 2
ax1 = ODrive_Ease_Lib.ODrive_Axis(od1.axis1)
ax1.index_and_hold(-1, 1)
time.sleep(3)
# Second ODrive controller
# od.axis0 --> motor 3
ax2 = ODrive_Ease_Lib.ODrive_Axis(od2.axis0)
ax2.index_and_hold(-1, 1)
time.sleep(3)



#create homing sequence with use of endstops
#using RPi.GPIO over ODrive endstops

# get position of arms at limit switch
# move arms up slowly until switch is triggered

#homing motor 1
ax0.axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
set_point = ax0.axis.controller.pos_setpoint
lim1_status = getLim1()
while (lim1_status == False):
    set_point -= 10
    ax0.axis.controller.pos_setpoint = set_point
    #print(set_point, lim1_status)
    time.sleep(0.5)
    lim1_status = getLim1()

ax0_home_position = set_point
print("homed Motor 1")
time.sleep(3)

#homing motor 2
ax1.axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
set_point = ax1.axis.controller.pos_setpoint
lim2_status = getLim2()
while (lim2_status == False):
    set_point -= 10
    ax1.axis.controller.pos_setpoint = set_point
    time.sleep(0.5)
    lim2_status = getLim2()

ax1_home_position = set_point
print("homed Motor 2")
time.sleep(3)

#homing motor 3
ax2.axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
set_point = ax2.axis.controller.pos_setpoint
lim3_status = getLim3()
while (lim3_status == False):
    set_point -= 10
    ax2.axis.controller.pos_setpoint = set_point
    time.sleep(0.4)
    lim3_status = getLim3() # uses 0.1 seconds
    # 0.4 + 0.1 = 0.5 --> 20 pulses per second. good slow homing speed

ax2_home_position = set_point
print("homed Motor 3")
time.sleep(3)


#delay shutdown for a minute
delay_time = 30 # seconds before rebooting
print("waiting", str(delay_time), "seconds")
time.sleep(delay_time)

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


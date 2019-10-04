import odrive
from RPi_ODrive import ODrive_Ease_Lib
import odrive.enums

# make sure to reboot odrive between script runs

od = odrive.find_any()
if not od:
    print("Did not find an ODrive")
    exit()

# Find way to separate two ODrives by ID

# First ODrive controller
'''
# od.axis0 --> motor 1
ax0 = ODrive_Ease_Lib.ODrive_Axis(od.axis0)
ax0.index_and_hold(-1, 1)    # works!!! up until this point

# od.axis1 --> motor 2
ax1 = ODrive_Ease_Lib.ODrive_Axis(od.axis1)
ax1.index_and_hold(-1, 1)   # works!!! up until this point
'''

# Second ODrive controller

# od.axis0 --> motor 3
ax2 = ODrive_Ease_Lib.ODrive_Axis(od.axis0)
ax2.index_and_hold(-1, 1)

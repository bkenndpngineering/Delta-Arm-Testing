import odrive
from RPi_ODrive import ODrive_Ease_Lib
import odrive.enums

o = ODrive_Ease_Lib.find_ODrives()

try:
    o[1].reboot()
except:
    pass
try:
    o[0].reboot()
except:
    pass

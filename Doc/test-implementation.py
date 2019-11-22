# minimalistic script to test API functionality
from deltaArm import DeltaArm
import time

arm = DeltaArm()

print("Running initialization")
arm.initialize()
print("finished init")

print("rotating stepper")
arm.rotateStepper(180)
print("rotating stepper again")
arm.rotateStepper(-180)

print("solenoid true")
arm.powerSolenoid(True)
time.sleep(2)
print("solenoid false")
arm.powerSolenoid(False)

# kinematic test
print("moving to (0, 0, -680)")
arm.moveToCoordinates(0, 0, -680)
print("get coordinates test:", arm.getCoordinates())
print("moving to (0, 0, -830)")
arm.moveToCoordinates(0, 0, -830)
print("get coordinates test:", arm.getCoordinates())

# rapid sequential movement test
arm.moveToCoordinates(0, 0, -680)
arm.moveToCoordinates(0, 0, -750)
arm.moveToCoordinates(0, 0, -830)

print("shutoff in 5 seconds")
time.sleep(5)
arm.shutdown()
exit()

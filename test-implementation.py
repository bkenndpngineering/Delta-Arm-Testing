from deltaArm import DeltaArm
import time

arm = DeltaArm()

print("Running initialization in 5 seconds")
time.sleep(5)
arm.initialize()
time.sleep(5)
print("finished init")

"""
print("rotating stepper")
arm.rotateStepper(90)
time.sleep(2)
print("rotating stepper")
arm.rotateStepper(-90)
time.sleep(2)
"""
"""
while arm.getLim1() == False:
    pass
print("found lim 1")
while arm.getLim2() == False:
    pass
print("found lim 2")
while arm.getLim3() == False:
    pass
print("found lim 3")

print("solenoid true")
arm.powerSolenoid(True)
time.sleep(2)

print("solenoid false")
arm.powerSolenoid(False)
time.sleep(2)
"""
print("moving to (0, 0, -680)")
arm.moveToCoordinates(0, 0, -680)
time.sleep(5)

print("get coordinates test:", arm.getCoordinates())

print("moving to (0, 0, -830)")
arm.moveToCoordinates(0, 0, -830)
time.sleep(5)

print("get coordinates test:", arm.getCoordinates())

print("shutoff in 5 seconds")
time.sleep(5)
arm.shutdown()
exit()

from deltaArm import DeltaArm
import time

arm = DeltaArm()

print("Running initialization in 5 seconds")
time.sleep(5)
arm.initialize()
time.sleep(5)
print("finished init")

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

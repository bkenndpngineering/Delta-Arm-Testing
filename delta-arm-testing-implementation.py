from deltaArm import DeltaArm
import time

arm = DeltaArm()

print("Running initialization in 5 seconds")
time.sleep(5)

arm.initialize()
print("finished init")
print("running shutoff in 5 seconds")

arm.shutdown()
exit()

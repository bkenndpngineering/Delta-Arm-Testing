from DeltaArm.kinematicFunctions import *
import math

"""
theta1 = 0
theta2 = 0
theta3 = 0

# current angles of the motor --> error, does not change
phi_vals = [math.radians(210), math.radians(90), math.radians(330)]

(x, y, z) = forward_kinematics(theta1, theta2, theta3, phi_vals)
print((x, y, z))

(theta1, theta2, theta3) = compute_triple_inverse_kinematics(x, y, z, phi_vals)
print((theta1, theta2, theta3))

phi_vals[0] = theta1
phi_vals[1] = theta2
phi_vals[2] = theta3

(theta1, theta2, theta3) = compute_triple_inverse_kinematics(0,0,-500, phi_vals)
print((theta1, theta2, theta3))
"""

#phi_vals = [math.radians(210), math.radians(90), math.radians(330)]

while 1:
    x = float(input("x: "))
    y = float(input("y: "))
    z = float(input("z: "))

    print((x,y,z))
    (theta1, theta2, theta3) = compute_triple_inverse_kinematics(x,y,z)
    print((theta1, theta2, theta3))
    print((x,y,z))


    # angles cannot equal -90 degrees, or else it will break
    # happens at z = -310, x and y = 0

from deltaArm import DeltaArm

print("setting up")
arm = DeltaArm()
arm.initialize()

print("starting demo loop")
arm.moveToCoordinates(0, 0, -680)

# a star pattern
coordinate_list = [[3, 2],
                    [-3, 2],
                    [2, -2],
                    [0, 4],
                    [-2, -2]
                    ]
try:
    while 1:
        for coordinate in coordinate_list:
            arm.moveToCoordinates(coordinate[0]*50, coordinate[1]*50, -680)
except:
    pass

# shutdown procedure
arm.moveToCoordinates(0, 0, -680)
arm.moveToCoordinates(0, 0, -830)
arm.shutdown()
exit()

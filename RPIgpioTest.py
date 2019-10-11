import RPi.GPIO as GPIO

limit1 = 13
limit2 = 16
limit3 = 20

GPIO.setmode(GPIO.BCM)

GPIO.setup(limit1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(limit3, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while 1:
    if (not GPIO.input(limit3)): print("PUSHED")

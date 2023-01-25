import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)
pwn = GPIO.PWM(18, 50) # GPIO 17 for PWM with 50Hz
pwn.start(10)
while True:
            pwn.ChangeDutyCycle(9)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(8)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(7)
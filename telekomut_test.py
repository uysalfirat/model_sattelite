import time
import RPi.GPIO as GPIO
import time
import serial
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)
pwn = GPIO.PWM(18, 50) # GPIO 17 for PWM with 50Hz
pwn.start(7) # Initialization
ser = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_TWO,bytesize=serial.EIGHTBITS,timeout=1)
try:
    while True:
        time.sleep(1)
        gelen=ser.readline().strip()
        a=len(gelen)
        if(a==1):
            if(int(gelen[0])==49):
                pwn.ChangeDutyCycle(7)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(10)
                time.sleep(0.5)
                pwn.ChangeDutyCycle(8)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(11)
                time.sleep(0.5)
                pwn.ChangeDutyCycle(8)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(11)
                time.sleep(0.5)
                pwn.ChangeDutyCycle(7)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(10)
                time.sleep(0.5)
                pwn.ChangeDutyCycle(8)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(11)
                time.sleep(0.5)
                pwn.ChangeDutyCycle(8)
                time.sleep(0.5)
        pwn.ChangeDutyCycle(12)
        pwn.stop()
            #elif(int(gelen[0])==48):
                #pwn.ChangeDutyCycle(11)
                #time.sleep(0.5)
        elif(a==5):
            with open('/home/pi/Desktop/sifre.txt','a+') as sifre:
                sifre.write(str(gelen))  
        #print(gelen,a)
except IOError:
    pwn.stop()
    GPIO.cleanup()
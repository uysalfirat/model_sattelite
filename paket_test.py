import time
import sys
import RPi.GPIO as GPIO
import board
from datetime import datetime
import busio
import digitalio
import adafruit_bmp280
import time
import serial
import smbus
import math
from time import sleep


ser = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_TWO,bytesize=serial.EIGHTBITS,timeout=1)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.CE1)
sensor = adafruit_bmp280.Adafruit_BMP280_SPI(spi, cs)
router=0
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)
pwn = GPIO.PWM(18, 50)
hiz='8'
voltaj='17.5'
durum='Model uydu iniş'
turn=2
enlem='0.35'
boylam='0.43'
irtifa_gps='990'
Gx='0'
Gy='90'
Gz='0'
while 1:
    altitude=format(sensor.altitude)
    pressure=format(sensor.pressure)
    temperature=format(sensor.temperature)
    altitude_1=str(altitude[0:4])
    pressure_1=str(pressure[0:3])
    temperature_1=str(temperature[0:2])
    zam=datetime.now()
    zaman=str(zam)
    router_1=str(router)
    turner=str(turn)
    paket=str('44203' + ',' + router_1 + ',' + zaman + ',' + pressure_1 + ',' + altitude_1 + ',' + hiz + ',' + temperature_1 + ',' + voltaj + ',' + enlem + ',' + boylam + ',' + irtifa_gps + ',' + Gx + ',' + Gy + ',' + Gz + ',' + turner )
    ser.write(str.encode(paket))
    gelen=ser.readline().strip()
    a=len(gelen)
    print(paket)
    if(a==1):
        if(int(gelen[0])==49):
            pwn.start(10)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(9)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(8)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(7)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(7)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(7)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(12)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(8)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(13)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(7)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(12)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(8)
            time.sleep(0.5)
            pwn.ChangeDutyCycle(6)
            time.sleep(0.5)
            pwn.stop()
        elif(a==5):
            with open('/home/pi/Desktop/sifre.txt','a+') as sifre:
                sifre.write(str(gelen))
    router += 1
    turn += 1
    time.sleep(1)
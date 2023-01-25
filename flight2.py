                            #İMPORT ALL OF SENSOR
#BMP280(SPI)
import board
import busio
import digitalio
import adafruit_bmp280
#GPS NEO6M.V2
import serial
from time import sleep
import sys
#MPU6050(İ2C)
import smbus
import math
import time
#SERVO
import RPi.GPIO as GPIO
import time
#picamera
from picamera import PiCamera, Color
#zaman
from datetime import datetime
#sleep(60)

                        #DEFİNE ALL OF SENSORS
#picamera
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 20
camera.brightness = 60
camera.annotate_background = Color('blue')
camera.annotate_foreground = Color('yellow')
camera.annotate_text = "Hisar MUY"
camera.annotate_text_size = 20

#BMP280(SPI)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.CE1)
sensor = adafruit_bmp280.Adafruit_BMP280_SPI(spi, cs)

#voltage
AO_pin = 0
SPICLK = 21
SPIMISO = 19
SPIMOSI = 20
SPICS = 8

#MPU6050
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
def read_byte(adr):
    return bus.read_byte_data(address, adr)
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
def get_z_rotation(x,y,z):
    radians = math.atan2(z, dist(x,y))
    return math.degrees(radians)
bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, power_mgmt_1, 0)

#voltage
def init():
          GPIO.setwarnings(False)
          GPIO.setmode(GPIO.BCM)
          GPIO.setup(SPIMOSI, GPIO.OUT)
          GPIO.setup(SPIMISO, GPIO.IN)
          GPIO.setup(SPICLK, GPIO.OUT)
          GPIO.setup(SPICS, GPIO.OUT)
          pass
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
    if((adcnum > 7)or(adcnum < 0)):
          return -1
    GPIO.output(cspin, True)
    GPIO.output(clockpin, False)
    GPIO.output(cspin, False)
    commandout = adcnum
    commandout |= 0x18
    commandout <<= 3
    for i in range(5):
            if(commandout & 0x80):
                 GPIO.output(mosipin, True)
            else:
                GPIO.output(mosipin, False)
            commandout <<= 1
            GPIO.output(clockpin, True)
            GPIO.output(clockpin, False)
    adcout = 0
    for i in range(12):
            GPIO.output(clockpin, True)
            GPIO.output(clockpin, False)
            adcout <<= 1
            if (GPIO.input(misopin)):
                    adcout |= 0x1
    GPIO.output(cspin, True)
    adcout >>= 1
    return adcout

#GPS NEO6M.V2
def GPS_Info():
    global NMEA_buff
    global enlem
    global boylam
    global alti
    global nmea_altitude1
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_altitude = []
    nmea_time = NMEA_buff[0]
    nmea_latitude = NMEA_buff[1]
    nmea_altitude = NMEA_buff[8]
    nmea_longitude = NMEA_buff[3]
    nmea_altitude1=nmea_altitude[0:4]
    alti = int(nmea_altitude1)
gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")
GPGGA_buffer = 0
NMEA_buff = 0
enlem = 0
boylam = 0
irtifa = 0
alti = 0

#BUZZER
GPIO.setmode(GPIO.BCM)
buzzer=23
GPIO.setup(buzzer,GPIO.OUT)

#------picamera
camera.start_preview()
camera.start_recording('/home/pi/Desktop/hisar-kayıt-uydu.h264')
#------zaman

#SERVO
GPIO.setup(17, GPIO.OUT)
pwn = GPIO.PWM(17, 50)
durum = 'hazır'
i=1
kontrol=0
numara=44203
sayac=1
#XBEE
serx = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_TWO,bytesize=serial.EIGHTBITS,timeout=1)
#sıfırlanacak değerler
irtifa_bmp_0=format(sensor.altitude)[0:4]

while True:

    zam=datetime.now()
    zaman=str(zam)
    #------BMP280(SPI)
    irtifa_bmp=(format(sensor.altitude)-irtifa_bmp_0)
    basınc=format(sensor.pressure)
    sicaklik=format(sensor.temperature)
    #------MPU6050(İ2C)    
    gyro_xout=read_word_2c(0x43)
    gyro_yout=read_word_2c(0x45)
    gyro_zout=read_word_2c(0x47)
    Gx=gyro_xout/131
    Gy=gyro_yout/131
    Gz=gyro_zout/131
    #------voltage
    ad_value=readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
    voltage=ad_value*(3.3/205)+10
    #------GPS NEO6M.V2
    received_data = (str)(ser.readline())
    GPGGA_data_available = received_data.find(gpgga_info)
    if (GPGGA_data_available>0):
        GPGGA_buffer = received_data.split("$GPGGA,",1)[1]
        NMEA_buff = (GPGGA_buffer.split(','))
        GPS_Info()
    def main():
        init()
    irtifa_bmp1=irtifa_bmp[0:4]
    bmp_irtifa=int(irtifa_bmp1)
    altitude2=(bmp_irtifa + alti)/2
    hiz=(altitude2-altitude1)/1
    altitude1=altitude2
    paket=str(numara+','+sayac+','+zaman+','+basınc+' '+altitude2+' '+hiz+' '+sicaklik+' '+enlem+' '+boylam+' '+irtifa_gps+' '+durum+' '+Gx+' '+Gy+' '+Gz)
    #-------XBEE
    serx.write(str.encode(paket))
    sayac +=1
    time.sleep(1)
    gelen=serx.readline().strip()
    a=len(gelen)
    if(a==1):
        if(int(x[0])==49):
            pwn.ChangeDutyCycle(7)
            time.sleep(1)
        elif(int(x[0])==48):
            pwn.ChangeDutyCycle(10)
            time.sleep(1)
    elif(a==5):
        with open('/home/pi/Desktop/sifre.txt','a+') as sifre:
            sifre.write(str(x[0:5])) 
      
    
                                                        #PRİNT ALL OF RESULT
    print('Jiroskop X : ', gyro_xout, ' olcekli: ', (gyro_xout / 131), '\n''Jiroskop Y : ', gyro_yout, ' olcekli: ', (gyro_yout / 131) , '\n''Ivmeolcer X: ', accel_xout, ' olcekli: ', accel_xout_scaled ,'\n''Ivmeolcer Y: ', accel_yout, ' olcekli: ', accel_yout_scaled, '\n' 'Ivmeolcer Z: ', accel_zout, ' olcekli: ', accel_zout_scaled ,'\n''X dondurme: ' , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled),'\n''Y dondurme: ' , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled),'\n''Sıcaklık:          {} C derece',sicaklik, '\n''Basınç:            {}hPa',basinc, '\n''İrtifa:            {}m'.format(sensor.altitude), '\n'"lat in degrees:   ", lat_in_degrees,'\n' "long in degree:   ",long_in_degrees, '\n'"Hisar Model Uydumuz şu an:", durum,  '\n' " Voltage is: " + str("%.2f"%voltage)+"V" '\n' '------------------------------------------------')
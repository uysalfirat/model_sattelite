import board
import busio
import digitalio
import adafruit_bmp280
import serial
from time import sleep
import sys
import time
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.CE1)
sensor = adafruit_bmp280.Adafruit_BMP280_SPI(spi, cs)

def GPS_Info():
    
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    global alti_in_degrees
    global alti
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_altitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_altitude = NMEA_buff[8] 
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    nmea_altitude1=nmea_altitude[0:4]
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation  
    alti = int(nmea_altitude1)
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    alti_in_degrees = convert_to_degrees(alti)  ##get altitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")                         #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0
altitude1=0
paket=0

while True:
    received_data = (str)(ser.readline())                   #read NMEA string received
    GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
    if (GPGGA_data_available>0):
        bmpaltitude=format(sensor.altitude)
        GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
        NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
        GPS_Info()   
        altitudebmp=format(sensor.altitude)
        altitudebmp1=altitudebmp[0:4]
        bmpaltitude=int(altitudebmp1)
        altitude2=(bmpaltitude + alti)/2
        hiz=(altitude2-altitude1)*2
        altitude1=altitude2
        with open('/home/pi/Desktop/hiz.txt','a+') as sifre:
            sifre.write(str(paket))
            sifre.write(')')
            sifre.write(str(hiz))
            sifre.write(---)
        paket +=1
        time.sleep(0.5)
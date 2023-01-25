import serial
from time import sleep
import sys
def GPS_Info():  
    global NMEA_buff
    global nmea_latitude
    global nmea_altitude
    global nmea_longitude
    global nmea_time
    global alti
    global lati
    global longi
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_altitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_altitude = NMEA_buff[8]                #extract altitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    lati = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convert string into float for calculation
    alti = float(nmea_altitude)                 #convert string into float for calculation

gpgga_info = "$GPGGA"
ser = serial.Serial ("/dev/ttyAMA0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0

while True:
    received_data = (str)(ser.readline())                   #read NMEA string received
    GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string
    if (GPGGA_data_available>0):
        GPGGA_buffer = received_data.split("$GPGGA",1)[1]  #store data coming after "$GPGGA," string
        NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
        GPS_Info()
        print("Time: ", nmea_time, '\n')
        print("Latitude:", nmea_latitude, "Longitude:", nmea_longitude, 'Altitude:', nmea_altitude, '\n')
    #def main():
        #init()

import serial
from time import sleep
gpgga_info="$GPGGA"
ser=serial.Serial("/dev/ttyS0")                          #Open port with baud rate
GPGGA_buffer=0
NMEA_buff=0
nmea_time=[]
nmea_latitude=[]
nmea_longitude=[]
nmea_altitude=[]
while True:
    received_data=(str)(ser.readline())                   #read NMEA string received
    GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
    if (GPGGA_data_available>0):
        GPGGA_buffer=received_data.split("$GPGGA",1)[1]     #store data coming after "$GPGGA," string 
        NMEA_buff=(GPGGA_buffer.split(','))                 #store comma separated data in buffer
        nmea_time=NMEA_buff[1]
        nmea_latitude=NMEA_buff[2]
        nmea_altitude=NMEA_buff[9]
        nmea_longitude=NMEA_buff[4]
        print(" Time      : ",nmea_time,'\n',"Latitude  : ", nmea_latitude,'\n',"Longitude : ", nmea_longitude,'\n','Altitude  : ', nmea_altitude,'\n',"------------------------------------------------------------\n")
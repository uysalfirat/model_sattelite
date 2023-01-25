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
#SERVO
import RPi.GPIO as GPIO
import time
#picamera
from picamera import PiCamera, Color
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
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value - 65536
        return value
bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

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
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)
        GPIO.output(clockpin, False)
        GPIO.output(cspin, False)
        commandout = adcnum
        commandout |= 0x18
        commandout <<= 3
        for i in range(5):
                if (commandout & 0x80):
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

#GPSNEO6M.V2
def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]
    nmea_latitude = NMEA_buff[1]
    nmea_altitude = NMEA_buff[8]
    nmea_longitude = NMEA_buff[3] 
    lat = float(nmea_latitude)
    longi = float(nmea_longitude)  
    lat_in_degrees = convert_to_degrees(lat)
    long_in_degrees = convert_to_degrees(longi)
  
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position    
gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

#BUZZER
GPIO.setmode(GPIO.BCM)
buzzer=23
GPIO.setup(buzzer,GPIO.OUT)

#SERVO 
GPIO.setup(17, GPIO.OUT)
pwn = GPIO.PWM(17, 50)
durum = 'göklerde'
i=1
kontrol=0
#XBEE
serx = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_TWO,bytesize=serial.EIGHTBITS,timeout=1)
paket=0


while True:
    #picamera
    camera.start_preview()
    camera.start_recording('/home/pi/Desktop/hisar-kayıt-uydu.h264')
    
    
    #BMP280(SPI)
    altitude=format(sensor.altitude)
    pressure=format(sensor.pressure)
    temperature=format(sensor.temperature)

    #GPS NEO6M.V2
    received_data = (str)(ser.readline())
    GPGGA_data_available = received_data.find(gpgga_info)
    if (GPGGA_data_available>0):
        GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #GPS
        NMEA_buff = (GPGGA_buffer.split(','))               #GPS
        GPS_Info()
    def main():
        init()
    #MPU6050(İ2C)    
    Ax=read_raw_data(ACCEL_XOUT_H)/16384
    Ay=read_raw_data(ACCEL_YOUT_H)/16384
    Az=read_raw_data(ACCEL_ZOUT_H)/16384
    Gx=read_raw_data(GYRO_XOUT_H)/131
    Gy=read_raw_data(GYRO_YOUT_H)/131
    Gz=read_raw_data(GYRO_ZOUT_H)/131     
    #voltage
    ad_value=readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
    voltage0=ad_value*(3.3/1024)*5
    voltage=10+voltage0   
    #XBEE
    serx.write(str.encode('Write router: %d \n'%(paket),"Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz,u'\u00b0'+"/s",'\n'"\tAx=%.2f g"%Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az,'\n''Sıcaklık:          {} C derece'.format(sensor.temperature), '\n''Basınç:            {}hPa'.format(sensor.pressure), '\n''İrtifa:            {}m'.format(sensor.altitude), '\n'"lat in degrees:   ", lat_in_degrees,'\n' "long in degree:   ",long_in_degrees,'Altitude:', nmea_altitude, '\n'"Hisar Model Uydumuz şu an:", durum, '\n''------------------------------------------------','\n'))
    time.sleep(1)
    paket +=1
    x=serx.readline().strip(),e
      
    
                                                        #PRİNT ALL OF RESULT
    print("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz,u'\u00b0'+"/s",'\n'"\tAx=%.2f g"%Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az,'\n''Sıcaklık:          {} C derece'.format(sensor.temperature), '\n''Basınç:            {}hPa'.format(sensor.pressure), '\n''İrtifa:            {}m'.format(sensor.altitude), '\n'"lat in degrees:   ", lat_in_degrees,'\n' "long in degree:   ",long_in_degrees, '\n'"Hisar Model Uydumuz şu an:", durum,  '\n' " Voltage is: " + str("%.2f"%voltage)+"V" '\n' '------------------------------------------------')

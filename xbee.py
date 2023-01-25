import time
from datetime import datetime
import serial
ser = serial.Serial(port='/dev/ttyUSB0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_TWO,bytesize=serial.EIGHTBITS,timeout=1)
router=0
while 1:
    #ser.write(str.encode("23,4,45,56,67,78,89,98,90,21,32,34,45,43,67,87"))
    #ser.write(1)
    #paket=str(router+345 + 567 + 678)
    #ser.write(str.encode(paket))
    zam=datetime.now()
    paket_num=str(router)
    zaman= (zam.year , zam.month , zam.day , zam.hour , zam.minute , zam.second)
    zamane=str(zaman)
    paket=str('44203' + ',' + paket_num + ',' + zamane + ',' + '23.5' + ',' + '43.9' + ',' + '45.8'+','+ '23' + ',' + '43' + ',' + '45'+ ','+'23' + ',' + '43' + ',' + '45'+','+ '23' + ',' + '43' + ',' + '45'+ ','+'23')
    ser.write(str.encode(paket))
    router += 1
    x=ser.readline().strip()
    print(paket,x)
    time.sleep(1)
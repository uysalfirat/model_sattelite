import RPi.GPIO as GPIO
import time
AO_pin = 0
SPICLK = 21
SPIMISO = 19
SPIMOSI = 20
SPICS = 8
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
def main():
         init()
         time.sleep(2)
         while True:
                  ad_value = readadc(AO_pin, SPICLK, SPIMOSI, SPIMISO, SPICS)
                  voltage0=ad_value*(3.3/1024)*5
                  voltage=8.5+voltage0
                  print (" Voltage is: " + str("%.2f"%voltage)+"V")
                  time.sleep(2)                  

if __name__ =='__main__':
         try:
                  main()
         except KeyboardInterrupt:
                  pass
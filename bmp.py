import board
import busio
import digitalio
import adafruit_bmp280
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.CE1)
sensor = adafruit_bmp280.Adafruit_BMP280_SPI(spi, cs)
print('Temperature: {} degrees C'.format(sensor.temperature))
print('Pressure: {} hPa'.format(sensor.pressure))
print('altitude: {} m'.format(sensor.altitude))
altitude=format(sensor.altitude)
pressure=format(sensor.pressure)
temperature=format(sensor.temperature)
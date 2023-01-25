from picamera import PiCamera, Color
from time import sleep
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 20
camera.brightness = 60
camera.annotate_background = Color('green')
camera.annotate_foreground = Color('red')
camera.annotate_text = "Hisar MUY"
camera.annotate_text_size = 20
camera.awb_mode = 'sunlight' #You can use camera.awb_mode to set the auto white balance to a preset mode.
camera.start_preview()
camera.start_recording('/home/pi/Desktop/video.h264')
sleep(20)
camera.stop_recording()
camera.stop_preview()
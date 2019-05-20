from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.vflip = True
camera.start_preview()#fullscreen=False, window=(100,200,300,400))
print("Input fname")
fname = input()
print("Taken")
camera.capture('%s.jpg' % fname)
camera.stop_preview()

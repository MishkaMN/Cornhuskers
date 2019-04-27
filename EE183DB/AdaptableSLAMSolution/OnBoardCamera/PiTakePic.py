from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview(fullscreen=False, window=(100,200,300,400))
for i in range(5,15):
    print("MOVE")
    sleep(5)
    print("Taken")
    camera.capture('cal%s.jpg' % i)
camera.stop_preview()

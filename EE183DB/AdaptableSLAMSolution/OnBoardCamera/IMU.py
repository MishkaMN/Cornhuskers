import FaBo9Axis_MPU9250
import numpy as np
import time

imu = FaBo9Axis_MPU9250.MPU9250()

def readAngle():
	mag = imu.readMagnet()
	offx = -13
	offy = 17
	#print(mag)
	return(np.arctan2(mag['y'] - 17,  mag['x'] +13))


if __name__ == "__main__":
	while True:
		#mag = imu.readMagnet()
		#print(mag['x'], mag['y'])
		print(readAngle())
		time.sleep(0.15)

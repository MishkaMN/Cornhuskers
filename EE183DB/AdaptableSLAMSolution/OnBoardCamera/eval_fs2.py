import numpy as np
import math
import matplotlib.pyplot as plt
import fs2w
import time
use_history = False
show_animation = False

def main():
	a= input("Use history? \n")
	if (a == 'y'):
		use_history = True
		show_animation = False
	else:
		use_history = False
		show_animation = True
	# initializaiton
	num_particle = 10
	distH = np.zeros((10,15))
	angleH = np.zeros((10,15))
	timeH = np.zeros((10,15))

	legends = ['.1', '.2','.3','.4', '.5','.6','.7', '.8','.9','1.0']
	#plt.figure()
	dts = np.linspace(.1, 1, 10)
	
	fig, ax = plt.subplots()
	angle = np.zeros((1,1))
	cnt = 0
	if use_history:
		distH = np.load("distH.npy")
		angleH = np.load("angleH.npy")
		timeH = np.load("timeH.npy")
		plt.cla()
		for i in range(10):
			plt.plot(np.linspace(10,100,10),distH[i,:10])
		plt.xlabel('Num of Particles')
		plt.ylabel('Error in Distance, mm')
		plt.legend(legends)
		plt.savefig('Distance_Error.png')
		plt.cla()
		for i in range(10):
			plt.plot(np.linspace(10,100,10),angleH[i,:10])
		plt.xlabel('Num of Particles')
		plt.ylabel('Error in Angle, deg')
		plt.legend(legends)
		plt.savefig('Angle_Error.png')
		plt.cla()
		for i in range(10):
			plt.plot(np.linspace(10,100,10),timeH[i,:10])
		plt.xlabel('Num of Particles')
		plt.ylabel('Process Time, s')
		plt.legend(legends)
		plt.savefig('Process_Time.png')
	else:
		for i,dt in enumerate(dts):
			# 10 plots on 1 figure
			if i == 10:
				cnt = 1
				plt.cla()
			j = 0
			while num_particle < 152:
				dist, angle, total_time = fs2w.run(num_particle, dt)
				#np.random.rand(1) * 10
				
				distH[i,j] = dist
				angleH[i,j] = angle
				timeH[i,j] = total_time
				num_particle += 10
				j = j + 1;
	
			num_particle = 10
			"""
			plt.plot(np.linspace(10,150,15),distH[i,:])
			plt.xlabel('Num of iteration')
			plt.ylabel('Error in Distance, mm')
			plt.legend(legends[:i+1])
			plt.savefig('Distance_Error%d.png' % cnt)
			plt.pause(0.001)
			"""
		np.save("distH", distH)
		np.save("angleH", angleH)
		np.save("timeH", timeH)

if __name__ == '__main__':
    main()
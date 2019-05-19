import numpy as np
import math
import matplotlib.pyplot as plt
import fs2
import time

show_animation = True

def main():

	num_particle = 10
	distH = np.zeros((20,15))
	angleH = np.zeros((20,15))
	legends = ['.1', '.2','.3','.4', '.5','.6','.7', '.8','.9','1.0', 
		'1.1','1.2','1.3', '1.4','1.5','1.6', '1.7','1.8','1.9', '2.0']
	#plt.figure()
	dts = np.linspace(.1, 2, 20)
	
	for i,dt in enumerate(dts):
		plt.figure()
		j = 0
		while num_particle < 152:
			dist, angle = fs2.run(num_particle, dt)
			
			distH[i,j] = dist
			angleH[i,j] = angle
			num_particle += 10
			j = j + 1;
			print(distH, flush=True)
			#input()
		
		num_particle = 10
		plt.plot(np.linspace(10,150,15),distH[i,:])
		plt.xlabel('Num of iteration')
		plt.ylabel('Error in Distance, mm')
		plt.legend(legends[:i])
		plt.savefig('Distance_Error.png')
		#plt.pause(0.00001)

	#plot()
	#plt.cla()
	#plt.plot(distH[0,:])
	plt.xlabel('Num of iteration')
	plt.ylabel('Error in Distance, mm')
	plt.legend(legends)
	plt.savefig('Distance_Error.png')
	plt.show()
	#plt.cla()
	#plt.plot(angleH)
	#plt.xlabel('Num of iteration')
	#plt.ylabel('Error in Angle, deg')

	#plt.savefig('Angle_Error.png')

if __name__ == '__main__':
    main()
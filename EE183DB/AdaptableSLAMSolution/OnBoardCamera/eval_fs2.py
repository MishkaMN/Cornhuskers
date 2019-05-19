import numpy as np
import math
import matplotlib.pyplot as plt
import fs2w
import time

show_animation = True

def main():
	num_particle = 10
	distH = np.zeros((20,15))
	angleH = np.zeros((20,15))
	timeH = np.zeros((20,15))

	legends = ['.1', '.2','.3','.4', '.5','.6','.7', '.8','.9','1.0', 
		'1.1','1.2','1.3', '1.4','1.5','1.6', '1.7','1.8','1.9', '2.0']
	#plt.figure()
	dts = np.linspace(.1, 2, 20)
	
	fig, ax = plt.subplots()
	angle = np.zeros((1,1))
	cnt = 0
	
	for i,dt in enumerate(dts):
		if i > 10:
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
		plt.plot(np.linspace(10,150,15),distH[i,:])
		plt.xlabel('Num of iteration')
		plt.ylabel('Error in Distance, mm')
		plt.legend(legends[:i+1])
		plt.savefig('Distance_Error%d.png' % cnt)
		plt.pause(0.001)
	np.save("distH", distH)
	np.save("angleH", angleH)
	np.save("timeH", timeH)
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
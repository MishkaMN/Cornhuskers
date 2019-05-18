import numpy as np
import math
import matplotlib.pyplot as plt
import fs2

def main():
	num_particle = 10
	distH = np.zeros((20,15))
	angleH = np.zeros((20,15))

	dts = np.linspace(.1, 2, 20)
	for i,dt in enumerate(dts):
		j = 0
		while num_particle < 152:
			dist, angle = fs2.run(num_particle, dt)
			distH[i,j] = dist
			angleH[i,j] = angle
			num_particle += 10
			j = j + 1;
			print(distH, flush=True)
			input()

	plt.cla()
	plt.plot(distH[0,:])
	plt.xlabel('Num of iteration')
	plt.ylabel('Error in Distance, mm')
	plt.savefig('Distance_Error.png')
	
	plt.cla()
	plt.plot(angleH)
	plt.xlabel('Num of iteration')
	plt.ylabel('Error in Angle, deg')

	plt.savefig('Angle_Error.png')

if __name__ == '__main__':
    main()
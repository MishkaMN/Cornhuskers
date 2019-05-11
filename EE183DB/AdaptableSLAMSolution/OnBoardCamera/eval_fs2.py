import numpy as np
import math
import matplotlib.pyplot as plt
import fs2

def main():
	num_particle = 10
	distH = []
	angleH = []

	while num_particle < 152:
		dist, angle = fs2.run(num_particle)
		distH.append(dist)
		angleH.append(angle)
		num_particle += 10
	plt.cla()
	plt.plot(distH)
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
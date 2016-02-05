import numpy as np 
from scipy import io
import matplotlib.pyplot as plt

X, Y = [],[]
x, y = 3.0, 7.0
plt.scatter([x],[y],color='red')
theta = 0
v=0.1

goals = [
(5.87224, 7.98479),
(8.28473, 6.72965),
(6.67344, 5.20612),
(8, 3)
]
gx, gy = goals[0][0],goals[0][1]
plt.scatter([g[0] for g in goals],[g[1] for g in goals],color='green')

L=0.1

phi=0
phi_range = np.arange(-1,1,0.1)
obj = lambda phi,v,L,theta, target_theta: (theta+(v/L)*np.tan(phi)-target_theta)**2

def log_search(v,L,theta, target_theta):

	phi = 0
	bound = 0.5
	while bound > 0.001:

		if np.power(theta+(v/L)*np.tan(phi+bound)-target_theta,2)<\
		 np.power(theta+(v/L)*np.tan(phi)-target_theta,2):
			phi = phi+bound
		elif np.power(theta+(v/L)*np.tan(phi-bound)-target_theta,2)<\
		 np.power(theta+(v/L)*np.tan(phi)-target_theta,2):
			phi = phi-bound

		bound /= 2.0

	return phi

delta = 0.001
i = 0
for _ in range(30000):

	gx, gy = goals[i][0]-x,goals[i][1]-y

	#Target orientation
	target_theta = np.arctan(gy/gx)
	if gx<0: target_theta -= np.pi


	#Find the optimal phi
	phi = log_search(v,L,theta, target_theta)

	theta += delta * (v/L) * np.tan(phi)

	v=1-0.8*np.abs(phi)

	x += delta * v * np.cos(theta)
	y += delta * v * np.sin(theta)

	X.append(x)
	Y.append(y)

	if np.sqrt(gx**2+gy**2)<0.2:
		i+=1
		if i == len(goals): break
		gx, gy = goals[i][0],goals[i][1]

	# if i == 1:
	# 	print 'Position: {0},{1}'.format(x,y)
	# 	print 'WayPoint: {0}, {1}'.format(gx,gy)
	# 	print 'Target theta: {0}'.format(target_theta)
	# 	print 'Phi: {0}'.format(phi)
	# 	print 'Theta: {0}'.format(theta)
	# 	print 'v: {0}\n'.format(v)

#Draw map
obstacle_map = io.loadmat('./Maps/polygObst.mat')
N = len(obstacle_map['button'])

start = obstacle_map['startPos'][0]/10.0
goal = obstacle_map['goalPos'][0]/10.0

polygons = [start,goal,np.zeros(2)]
for n in range(N):

	if obstacle_map['button'][n] == 1:
		polygons.append(np.array([obstacle_map['x'][n][0]/10.0,
			obstacle_map['y'][n][0]/10.0]))
	else:
		polygons.append(np.zeros(2))


# Draw the map
plt.scatter([polygons[0][0]],polygons[0][1],color='red')
plt.scatter([polygons[1][0]],polygons[1][1],color='green')
line = [i/1000.0 for i in range(1000)]
i = 2
while i < len(polygons):
	nodes = []
	while 1:
		node = polygons[i]
		if all(node == np.zeros(2)): break
		nodes.append(node)
		i+=1

	for j in range(len(nodes)):

		if j+1 < len(nodes):
			n1, n2 = nodes[j], nodes[j+1]
		else:
			n1, n2 = nodes[j], nodes[0]

		plt.plot([(1-l)*n1[0]+l*n2[0] for l in line],
			[(1-l)*n1[1]+l*n2[1] for l in line], color='blue')

		plt.scatter([n[0] for n in nodes],[n[1] for n in nodes])

	i+=1

plt.plot(X,Y)
plt.show()
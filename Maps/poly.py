import numpy as np
from scipy import io
from bs4 import BeautifulSoup as bs
from Tkinter import Tk
from tkFileDialog import askopenfilename
import matplotlib.pyplot as plt

Tk().withdraw() 
filename = askopenfilename()

obstacle_map = io.loadmat(filename)
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

#txt Map
np.savetxt("{0}.txt".format(filename.split('/')[-1].split('.')[0]),
 polygons, delimiter=" ")

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

# nodes = [
# ]

# for node in nodes:
#   plt.scatter([node[1]],[node[2]],color='green')
#   plt.annotate(int(node[0]),(node[1],node[2]))


# graph = [

# ]

# for edge in graph:
# 	n1,n2 = edge
# 	plt.plot([(1-l)*n1[0]+l*n2[0] for l in line],
# 			[(1-l)*n1[1]+l*n2[1] for l in line], color='green')

# centers =[np.array([1.28648, 6.97368]),
# np.array([4.37558, 5.48246]),
# np.array([8.69816, 4.654]),
# np.array([6.27112, 1.95419]),
# np.array([2.37327, 2.38304])]

# plt.scatter([c[0] for c in centers],[c[1] for c in centers],color='orange')

# triangles =[
# ]

# plt.scatter([t[0] for t in triangles],[t[1] for t in triangles],color='orange')

# path = [
# np.array([18, 7.5]),
# np.array([10.1148, 9.39686]),
# np.array([4.90182, 25.0733]),
# np.array([3, 7])
# ]

# for i in range(len(path)-1):
#   n1 = path[i]
#   n2 = path[i+1]
#   plt.plot([(1-l)*n1[0]+l*n2[0] for l in line],
#       [(1-l)*n1[1]+l*n2[1] for l in line], color='green')


# plt.show()
# plt.close()

#Gazebo world
world = '''
<?xml version="1.0"?> 
<sdf version="1.4">
  <world name="default">

  <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
'''
bs_world = bs(world,'xml')

#Model
Tk().withdraw() 
model_filename = askopenfilename(initialdir="../Models")
with open(model_filename, 'r') as f:
  model = bs(f,'xml')

bs_world.include.insert_after(model.model)

z = str(bs_world.model.pose).split()[2]
bs_world.model.pose.replaceWith('<pose>{0} {1} {2} 0 0 0</pose>'\
.format(start[0],start[1],z))

#Target
with open('../Models/target_marble.model', 'r') as f:
  target = bs(f,'xml')

bs_world.include.insert_after(target.model)

z = str(bs_world.model.pose).split()[2]
bs_world.model.pose.replaceWith('<pose>{0} {1} {2} 0 0 0</pose>'\
.format(goal[0],goal[1],z))

coke_can = '''
<model name="coke_can">
	<static>1</static>
    <link name="link">
      <collision name="collision">
        <pose>{0} {1} -0.46 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://coke_can/meshes/coke_can.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>{0} {1} -0.46 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://coke_can/meshes/coke_can.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
'''
line = [i/20.0 for i in range(20)]
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


		for point in [(1-l)*n1+l*n2 for l in line]:

			bs_coke_can = bs(coke_can.format(point[0],point[1]),'xml')
			bs_world.include.insert_after(bs_coke_can.model)

	i+=1

with open('../Worlds/{0}.world'.format(filename.split('/')[-1].split('.')[0]),'wb') as f:
	f.write(bs_world.prettify().replace('&lt;','<').replace('&gt;','>'))
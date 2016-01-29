import numpy as np
from scipy import io
from bs4 import BeautifulSoup as bs
from Tkinter import Tk
from tkFileDialog import askopenfilename
import matplotlib.pyplot as plt

def angle(v1, v2):
  return np.arccos(v1.dot(v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

Tk().withdraw() 
filename = askopenfilename()

obstacle_map = io.loadmat(filename)
N = len(obstacle_map['button'])

polygons = [obstacle_map['startPos'][0]/10.0,
obstacle_map['goalPos'][0]/10.0,np.zeros(2)]
for n in range(N):

	if obstacle_map['button'][n] == 1:
		polygons.append(np.array([obstacle_map['x'][n][0]/10.0,
			obstacle_map['y'][n][0]/10.0]))
	else:
		polygons.append(np.zeros(2))

#txt Map
np.savetxt("polyObst.txt", polygons, delimiter=" ")

#Draw the map
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
			[(1-l)*n1[1]+l*n2[1] for l in line])

		plt.scatter([n[0] for n in nodes],[n[1] for n in nodes])

	i+=1

plt.show()
plt.close()

# #Gazebo world
# world = '''
# <?xml version="1.0"?> 
# <sdf version="1.4">
#   <world name="default">

#   <!-- Ground Plane -->
#     <include>
#       <uri>model://ground_plane</uri>
#     </include>

#     <include>
#       <uri>model://sun</uri>
#     </include>
#   </world>
# </sdf>
# '''
# bs_world = bs(world,'xml')

# #Model
# Tk().withdraw() 
# filename = askopenfilename(initialdir="../Models")
# with open(filename, 'r') as f:
#   model = bs(f,'xml')

# bs_world.include.insert_after(model.model)

# #Target
# with open('../Models/target_marble.model', 'r') as f:
#   target = bs(f,'xml')

# bs_world.include.insert_after(target.model)

# polygon_xml ='''
# <model name='polygon'>
#     <pose frame=''>{0} {1} 0 0 -0 0</pose>
# <static>1</static>
#   </model>
# '''

# wall_xml = '''
# <link name='Wall'>
#       <collision name='Wall_Collision'>
#         <geometry>
#           <box>
#             <size>{3} 0.15 2.5</size>
#           </box>
#         </geometry>
#         <pose frame=''>0 0 1.25 0 -0 0</pose>
#       </collision>
#       <visual name='Wall_Visual'>
#         <pose frame=''>0 0 1.25 0 -0 0</pose>
#         <geometry>
#           <box>
#             <size>{3} 0.15 2.5</size>
#           </box>
#         </geometry>
#         <material>
#           <script>
#             <uri>file://media/materials/scripts/gazebo.material</uri>
#             <name>Gazebo/Bricks</name>
#           </script>
#           <ambient>1 1 1 1</ambient>
#         </material>
#       </visual>
#       <pose frame=''>{0} {1} 0 0 0 {2}</pose>
#     </link>
# '''

# i=0
# while i<len(polygons):

# 	nodes = []
# 	while 1:
# 		node = polygons[i]
# 		if all(node==np.zeros(2)): break
# 		nodes.append(node)
# 		i+=1

# 	center = np.mean(nodes,axis=0)
# 	polygon = bs(polygon_xml.format(center[0],center[1]),'xml')

# 	for j in range(len(nodes)):

# 		if j+1 < len(nodes):
# 			n1, n2 = nodes[j], nodes[j+1]
# 		else:
# 			n1, n2 = nodes[j], nodes[0]

# 		n1, n2 = n1-center, n2-center

# 		m = (n1+n2)/2.0
# 		length = np.linalg.norm(n1-n2)

# 		wall = bs(wall_xml.format(m[0],m[1],
# 			-np.sign(n1[0]-m[0])*angle(n1-m,np.array([1,0])),length),'xml')

# 		polygon.pose.insert_after(wall)

# 	bs_world.include.insert_after(polygon)
# 	i+=1

# with open('../Worlds/polyObst.world','wb') as f:
# 	f.write(bs_world.prettify())
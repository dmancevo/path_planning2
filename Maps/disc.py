from scipy import io
from bs4 import BeautifulSoup as bs
from Tkinter import Tk
from tkFileDialog import askopenfilename

Tk().withdraw() 
filename = askopenfilename()

obstacle_map = io.loadmat(filename)['A']

rows,cols = obstacle_map.shape

#txt Map
txt = str(obstacle_map)[1:-1].replace(' [','').replace('[','').replace(']','')

with open('dicsObst.txt', 'w') as f:
  f.write(txt)

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
filename = askopenfilename(initialdir="../Models")
with open(filename, 'r') as f:
  model = bs(f,'xml')

bs_world.include.insert_after(model.model)

#Target
with open('../Models/target_marble.model', 'r') as f:
  target = bs(f,'xml')

bs_world.include.insert_after(target.model)

obstacle_xml = '''
<model name="box">
      <pose>{0} {1} 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
</model>
'''

for x in range(obstacle_map.shape[0]):
	for y in range(obstacle_map.shape[1]):

		if obstacle_map[x][y] == 0: continue

		obstacle = bs(obstacle_xml.format(x,y),'xml')

		bs_world.include.insert_after(obstacle)

with open('../Worlds/dicsObst.world','wb') as f:
	f.write(bs_world.prettify())
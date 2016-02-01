from bs4 import BeautifulSoup as bs
from Tkinter import Tk
from tkFileDialog import askopenfilename
from waypoint_data import *

txt = str(acc) + "\n" + str(vel)
for point in points:
  txt += "\n" + str(point[0]) + " " + str(point[1])

with open('waypoint.txt', 'w') as f:
  f.write(txt)

#Gazebo world
world = '''
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

point_xml = '''
<model name="point">
      <pose>{0} {1} 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>.2</radius>
              <length>.005</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>.2</radius>
              <length>.005</length>
            </cylinder>
          </geometry>
          <material>
          <script>
           <uri>
            file://media/materials/scripts/gazebo.material
           </uri>
           <name>
            Gazebo/Red
           </name>
          </script>
         </material>
        </visual>
      </link>
</model>
'''
for point in points:
    obstacle = bs(point_xml.format(point[0],point[1]),'xml')
    bs_world.include.insert_after(obstacle)

with open('../Worlds/waypointDynamic.world','wb') as f:
	f.write(bs_world.prettify())
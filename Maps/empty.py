from scipy import io
from bs4 import BeautifulSoup as bs
from Tkinter import Tk
from tkFileDialog import askopenfilename

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

with open('../Worlds/empty.world','wb') as f:
	f.write(bs_world.prettify())
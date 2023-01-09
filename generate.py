import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("box.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = 0.5


for ix in range(5):
  length = 1
  width = 1
  height = 1
  z = 0.5
  y = 0
  for yi in range(5):
    length = 1
    width = 1
    height = 1
    z = 0.5
    for i in range(10):
      oldHeight = height
      pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length, width, height])
      length = length*0.9
      width = width*0.9
      height = height*0.9
      z += (height/2) + (oldHeight/2) 
    y += 1
  x += 1

pyrosim.End()

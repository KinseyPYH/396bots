import pyrosim.pyrosim as pyrosim



length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5



def Create_World():
  pyrosim.Start_SDF("world.sdf")
  # pyrosim.Send_Cube(name="Box", pos=[x-3,y+3,z] , size=[length, width, height])
  pyrosim.End()


def Create_Robot():
  pyrosim.Start_URDF("body.urdf")
  pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
  pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
  pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length, width, height])

  pyrosim.End()

Create_World()
Create_Robot()

# for ix in range(5):
#   length = 1
#   width = 1
#   height = 1
#   z = 0.5
#   y = 0
#   for yi in range(5):
#     length = 1
#     width = 1
#     height = 1
#     z = 0.5
#     for i in range(10):
#       oldHeight = height
#       pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length, width, height])
#       length = length*0.9
#       width = width*0.9
#       height = height*0.9
#       z += (height/2) + (oldHeight/2) 
#     y += 1
#   x += 1


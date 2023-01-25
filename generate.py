import pyrosim.pyrosim as pyrosim
import random

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5



def Create_World():
  pyrosim.Start_SDF("world.sdf")
  pyrosim.Send_Cube(name="Box", pos=[x-3,y+3,z] , size=[length, width, height])
  pyrosim.End()


def Create_Robot():
  pyrosim.Start_URDF("body.urdf")
  pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
  pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
  pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length, width, height])

  pyrosim.End()

def Generate_Body():
  pyrosim.Start_URDF("body.urdf")
  pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
  pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length, width, height])
  pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
  pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length, width, height])

  pyrosim.End()

def Generate_Brain():
  pyrosim.Start_NeuralNetwork("brain.nndf")
  pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
  pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
  pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
  pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
  pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
  # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = 1.0 )
  # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = 1.0 )

  # pyrosim.Send_Synapse( sourceNeuronName = 2 , targetNeuronName = 4 , weight = -1.0 )
  # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 4 , weight = -1.0 )

  for i in range(3):
    # print(i)
    for j in range(3,5):
      pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = random.random()*2 - 1 )
      

  pyrosim.End()

Create_World()
# Create_Robot()
Generate_Body()
Generate_Brain()

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


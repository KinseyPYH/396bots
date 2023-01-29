import os
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
import time


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(3,2)
        self.weights = (self.weights * 2) - 1 
        self.myID = nextAvailableID
      
    def Evaluate(self, directOrGUI):
        self.Generate_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        f.close()
        print(self.fitness)
        os.system("rm " + fitnessFileName)
        
    def Start_Simulation(self, directOrGUI):
        self.Generate_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        print(self.fitness)
        f.close()
        os.system("rm " + fitnessFileName)

    def Create_World():
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[c.x-3,c.y+3,c.z] , size=[c.length, c.width, c.height])
        pyrosim.End()

    def Generate_Body():
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[c.length, c.width, c.height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[c.length, c.width, c.height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[c.length, c.width, c.height])

        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = 1.0 )
        # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = 1.0 )
        for currentRow in range(3):
            # print(i)
            for currentColumn in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn + 3 , weight = self.weights[currentRow][currentColumn])

        pyrosim.End()
    

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0,1)
        self.weights[randomRow, randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
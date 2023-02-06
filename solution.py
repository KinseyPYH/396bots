import os
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
import time


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = (self.weights * 2) - 1 
        self.myID = nextAvailableID
        self.Create_World()
        self.Create_Body()
      
    def Evaluate(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        print(f.readlines())
        self.fitness = [float(i) for i in f.readlines()[0].split(" ")]
        f.close()
        print(self.fitness)
        os.system("rm " + fitnessFileName)
        os.system("rm brain" + str(self.myID) + ".nndf")
        
    def Start_Simulation(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = [float(i) for i in f.readlines()[0].split(" ")]
        f.close()
        os.system("rm " + fitnessFileName)
        os.system("rm brain" + str(self.myID) + ".nndf")


    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        ## make stairs ##
        for i in range(6):
            pyrosim.Send_Cube(name="Box", pos=[c.x,c.y+i*c.width,(i*c.height)/2] , size=[c.length, c.width, i*c.height], mass=c.stairMass)
        pyrosim.Send_Cube(name="Box", pos=[c.x,c.y+(7.5*c.width),(6*c.height)/2] , size=[c.length, c.width*4, 6*c.height], mass=c.stairMass)
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1.9] , size=[c.body_length, c.body_width, c.body_height], mass=1)
        pyrosim.Send_Joint( name = "Torso_RightThigh" , parent= "Torso" , child = "RightThigh" , type = "revolute", position = [-(c.body_length/2+0.1),0,1.85], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightThigh", pos=[0,0,-0.3] , size=[0.2, 0.2, 0.6], mass=5)
        pyrosim.Send_Joint( name = "Torso_LeftThigh" , parent= "Torso" , child = "LeftThigh" , type = "revolute", position = [c.body_length/2+0.1,0,1.85], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LeftThigh", pos=[0,0,-0.3] , size=[0.2, 0.2, 0.6], mass=5)

        pyrosim.Send_Joint( name = "RightThigh_RightCalf" , parent= "RightThigh" , child = "RightCalf" , type = "revolute", position = [-0.2,-0.05,-0.6], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightCalf", pos=[0,0,-0.2] , size=[0.2, 0.2, 0.6], mass=50)
        pyrosim.Send_Joint( name = "LeftThigh_LeftCalf" , parent= "LeftThigh" , child = "LeftCalf" , type = "revolute", position = [0.2,-0.05,-0.6], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LeftCalf", pos=[0,0,-0.2] , size=[0.2, 0.2, 0.6], mass=50)

        pyrosim.Send_Joint( name = "RightCalf_RightFoot" , parent= "RightCalf" , child = "RightFoot" , type = "revolute", position = [0,0,-0.5], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightFoot", pos=[0,0.1,0], size=[0.5, 0.5, 0.05], mass=105)
        pyrosim.Send_Joint( name = "LeftCalf_LeftFoot" , parent= "LeftCalf" , child = "LeftFoot" , type = "revolute", position = [0,0,-0.5], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LeftFoot", pos=[0,0.1,0], size=[0.5, 0.5, 0.05], mass=105)

        pyrosim.End()


    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "RightThigh")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftThigh")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightCalf")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "LeftCalf")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "RightFoot")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LeftFoot")
        

        pyrosim.Send_Motor_Neuron(name = 7 , jointName = "Torso_RightThigh")
        pyrosim.Send_Motor_Neuron(name = 8 , jointName = "Torso_LeftThigh")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "RightThigh_RightCalf")
        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "LeftThigh_LeftCalf")
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = "RightCalf_RightFoot")
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = "LeftCalf_LeftFoot")
        

        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                if (currentColumn == 8):
                    self.weights[currentRow][currentColumn] = self.weights[currentRow][currentColumn-1]
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn + c.numSensorNeurons , weight = self.weights[currentRow][currentColumn])
        pyrosim.End()
    

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons-1)
        randomColumn = random.randint(0,c.numMotorNeurons-1)
        self.weights[randomRow, randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
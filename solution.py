import os
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
import time


class SOLUTION:
    def __init__(self, nextAvailableID):
        # self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        # self.weights = (self.weights * 2) - 1 
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
        self.fitness = float(f.read())
        f.close()
        print(self.fitness)
        os.system("rm " + fitnessFileName)
        
    def Start_Simulation(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID))# + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        print(self.fitness)
        f.close()
        os.system("rm " + fitnessFileName)

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[c.x-3,c.y+3,c.z] , size=[c.length, c.width, c.height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        self.snakeRandomSize = np.random.randint(3, high=10) #random num from 0 to 10

        self.snakeSensorColors = []
        self.numSensors = 0
        self.snakeJoints = []
        for i in range(self.snakeRandomSize):
            randomNum = np.random.rand()
            color = 'Blue'
            if randomNum > 0.5:
                color = 'Green'
                self.numSensors += 1
            self.snakeSensorColors.append(color)
        prevLength = np.random.rand() * c.maxSize
        prevWidth = np.random.rand() * c.maxSize
        prevHeight = np.random.rand() * c.maxSize
        pyrosim.Send_Cube(name="0", pos=[0,0,prevHeight/2 + c.maxSize] , size=[prevLength, prevWidth, prevHeight], color=self.snakeSensorColors[0])
        if (self.snakeRandomSize > 1): 
            self.snakeJoints.append(("0_1", np.random.rand()))
            pyrosim.Send_Joint( name = "0_1" , parent= "0" , child = "1" , type = "revolute", position = [prevLength/2,0,prevHeight/2 + c.maxSize], jointAxis = "0 1 0")
        
    
        for i in range(1,self.snakeRandomSize):
            prevLength = np.random.rand() * c.maxSize
            prevWidth = np.random.rand() * c.maxSize
            prevHeight = np.random.rand() * c.maxSize
            pyrosim.Send_Cube(name=str(i), pos=[prevLength/2,0,0] , size=[prevLength, prevWidth, prevHeight], color=self.snakeSensorColors[i])
            if (i < self.snakeRandomSize-1):
                jointName = str(i) + "_" + str(i+1)
                self.snakeJoints.append((jointName, np.random.rand()))
                pyrosim.Send_Joint( name = jointName , parent= str(i) , child = str(i+1) , type = "revolute", position = [prevLength,0,0], jointAxis = "0 1 0")

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        sensorNeuronCount = 0
        for i in range(self.snakeRandomSize):
            if self.snakeSensorColors[i] == 'Green':
                pyrosim.Send_Sensor_Neuron(name = sensorNeuronCount, linkName = str(i))
                sensorNeuronCount += 1 
            if sensorNeuronCount == 0 and i == self.snakeRandomSize - 1:
                pyrosim.Send_Sensor_Neuron(name = sensorNeuronCount, linkName = str(i))
                sensorNeuronCount += 1 


        c.numSensorNeurons = sensorNeuronCount
        motorNeuronCount = 0
        for i in range(len(self.snakeJoints)):
            if self.snakeJoints[i][1] > 0.5:
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.snakeJoints[i][0])
                motorNeuronCount += 1
            if motorNeuronCount == 0 and i == self.snakeRandomSize - 1:
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.snakeJoints[i][0])
                motorNeuronCount += 1

            
        c.numMotorNeurons = motorNeuronCount
        self.weights = np.random.rand(sensorNeuronCount, motorNeuronCount)

        self.weights = (self.weights * 2) - 1 
        for currentRow in range(sensorNeuronCount):
            for currentColumn in range(motorNeuronCount):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn + sensorNeuronCount , weight = 1)#self.weights[currentRow][currentColumn])
                
        pyrosim.End()
    

    def Mutate(self):
        # print(c.numSensorNeurons-1)
        numSensorNeurons = c.numSensorNeurons - 1
        numMotorNeurons = c.numMotorNeurons - 1

        if (c.numSensorNeurons == 1):
            numSensorNeurons = 0
        if (c.numMotorNeurons == 1):
            numMotorNeurons = 0
        randomRow = random.randint(0, numSensorNeurons)
        randomColumn = random.randint(0,numMotorNeurons)
        self.weights[randomRow, randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
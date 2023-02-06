import os
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy
import pybullet as p

class ROBOT:
    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        # os.system("rm brain" + str(solutionID) + ".nndf")
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.maxFitness = 0
        self.maxZ = 0
        self.maxY = 0


    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(t)
    
    def Prepare_To_Act(self):    
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, robot, i):

        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zCoordinateofLinkZero = basePosition[2]
        yCoordinateofLinkZero = basePosition[1]
        if self.maxFitness <= zCoordinateofLinkZero*0.35 + yCoordinateofLinkZero*0.65:
            self.maxFitness = zCoordinateofLinkZero*0.35 + yCoordinateofLinkZero*0.65
            self.maxZ = zCoordinateofLinkZero
            self.maxY = yCoordinateofLinkZero

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)*c.motorJointRange
                if (jointName == "Torso_RightThigh" or jointName == "Torso_LeftThigh"):
                    desiredAngle *= 0.9
                if (jointName == "LeftCalf_LeftFoot" or jointName == "RightCalf_RightFoot"):
                    desiredAngle *= 0.5
                self.motors[bytes(jointName, 'utf-8')].Set_Value(robot, desiredAngle)

    def Think(self):
        self.nn.Update()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        self.zCoordinateofLinkZero = basePosition[2]
        self.yCoordinateofLinkZero = basePosition[1]
        writeToFileName = "tmp" + str(self.solutionID) + ".txt"
        f = open(writeToFileName, "w")
        f.write(str(self.maxZ))
        f.write(" ")
        f.write(str(self.maxY))
        f.close()
        os.system("mv " + writeToFileName + " fitness" + str(self.solutionID) + ".txt")
import os
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy
import pybullet as p

class ROBOT:
    def __init__(self, seednum, solutionID, inputbrain_filename, inputbody_filename):
        self.solutionID = solutionID
        self.seednum = seednum
        self.foldername =  "seed%s" %(self.seednum)
        
        if inputbrain_filename:
            self.nn = NEURAL_NETWORK(inputbrain_filename)
            self.robotId = p.loadURDF(inputbody_filename)
        else:
            self.nn = NEURAL_NETWORK(self.foldername + "/brain" + str(solutionID) + ".nndf")
            bodyFileName = self.foldername + "/body" + str(solutionID) + ".urdf"
            self.robotId = p.loadURDF(bodyFileName)
        
        # self.robotId = p.loadURDF("body.urdf")

        pyrosim.Prepare_To_Simulate(self.robotId)


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
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)*c.motorJointRange
                self.motors[bytes(jointName, 'utf-8')].Set_Value(robot, desiredAngle)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        self.stateOfLinkZero = p.getLinkState(self.robotId,0)
        self.positionOfLinkZero = self.stateOfLinkZero[0]
        self.xCoordinateofLinkZero = self.positionOfLinkZero[0]
        print("Fitness: " + str(self.xCoordinateofLinkZero))
        writeToFileName = "tmp" + str(self.solutionID) + ".txt"
        f = open(writeToFileName, "w")
        f.write(str(self.xCoordinateofLinkZero))
        f.close()
        os.system("mv " + writeToFileName + " " + self.foldername + "/fitness" + str(self.solutionID) + ".txt")
        # exit()
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy

class ROBOT:
    def __init__(self):
        self.nn = NEURAL_NETWORK("brain.nndf")


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
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[bytes(jointName, 'utf-8')].Set_Value(robot, desiredAngle)

    def Think(self):
        self.nn.Update()
        self.nn.Print()
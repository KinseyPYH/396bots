import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy

class ROBOT:
    def __init__(self):
        pass

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for sensor in self.sensors:
            # print(type(sensor))
            # print(sensor)
            self.sensors[sensor].Get_Value(t)
    
    def Prepare_To_Act(self):
        self.amplitude = c.b_amplitude
        self.frequency = c.b_frequency
        self.offset = c.b_phaseOffset

        self.motors = {}
        
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
            for i in range(c.numIterations):
                if (jointName == b'Torso_BackLeg'):
                    self.motors[jointName].motorValues[i] = self.amplitude * numpy.sin(self.frequency/2 * c.x[i] + self.offset)
                else:
                    self.motors[jointName].motorValues[i] = self.amplitude * numpy.sin(self.frequency * c.x[i] + self.offset)

    def Act(self, robot, i):
        print(robot)
        for motor in self.motors:
            self.motors[motor].Set_Value(robot, i)
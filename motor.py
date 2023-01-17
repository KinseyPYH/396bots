import pyrosim.pyrosim as pyrosim
import pybullet as p

import numpy
import constants as c


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.motorValues = numpy.zeros(c.numIterations)

    def Set_Value(self, robot, i):
        pyrosim.Set_Motor_For_Joint( 
            bodyIndex = robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motorValues[i],
            maxForce = c.maxForceMotors)

    def Save_Values(self):
        numpy.save("data/" + self.jointName + ".npy", self.motorValues)

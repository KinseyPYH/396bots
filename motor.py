import pyrosim.pyrosim as pyrosim
import pybullet as p

import numpy
import constants as c


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.motorValues = numpy.zeros(c.numIterations)

    def Set_Value(self, robot, desiredAngle):

        if (self.jointName == "LeftCalf_LeftFoot" or self.jointName == "RightCalf_RightFoot"):
            pyrosim.Set_Motor_For_Joint( 
            bodyIndex = robot.robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle*0.8,
            maxForce = c.maxForceMotors*0.7)

        elif (self.jointName == "LeftThigh_LeftCalf" or self.jointName == "RightThigh_RightCalf"):
            pyrosim.Set_Motor_For_Joint( 
            bodyIndex = robot.robotId,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = c.maxForceMotors*1.5)
        else:
            pyrosim.Set_Motor_For_Joint( 
                bodyIndex = robot.robotId,
                jointName = self.jointName,
                controlMode = p.POSITION_CONTROL,
                targetPosition = desiredAngle,
                maxForce = c.maxForceMotors*0.8)

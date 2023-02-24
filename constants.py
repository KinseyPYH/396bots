import numpy

#Iterations and Time per step
numIterations = 5000
t = 1/500

# HillClimber parameters
numberOfGenerations = 1
populationSize = 1

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5
startHeight = 3


# Joints
maxNumLinks = 7

# Neurons and Motors
numSensorNeurons = 0
numMotorNeurons = 0

maxForceMotors = 100
motorJointRange = 1

maxSize = 0.75
minLinkSize = maxSize*0.5

import numpy

#Iterations and Time per step
numIterations = 6000
t = 1/100000

# HillClimber parameters
numberOfGenerations = 1
populationSize = 1

# torso dimensions
body_length = 0.6
body_width = 0.4
body_height = 0.3

# stair dimensions
length = 15
width = 1.5
height = 0.125
x = 0
y = 0
z = 0.25
stairMass=1000


# Neurons and Motors
numSensorNeurons = 7
numMotorNeurons = 6

maxForceMotors = 108
motorJointRange = 1.5
import numpy

numIterations = 2000
t = 1/10000
maxForceMotors = 100

#front
f_amplitude = numpy.pi/3
f_frequency = 10
f_phaseOffset = 0
#back
b_amplitude = numpy.pi/3
b_frequency = 10
b_phaseOffset = 0

x1 = numpy.linspace(0, 2*numpy.pi, numIterations)

numberOfGenerations = 1
populationSize = 1

length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5

numSensorNeurons = 9
numMotorNeurons = 8
motorJointRange = 0.2

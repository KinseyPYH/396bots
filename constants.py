import numpy

numIterations = 1000
t = 1/60
maxForceMotors = 300

#front
f_amplitude = numpy.pi/3
f_frequency = 10
f_phaseOffset = 0
#back
b_amplitude = numpy.pi/3
b_frequency = 10
b_phaseOffset = 0

x = numpy.linspace(0, 2*numpy.pi, numIterations)


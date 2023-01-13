import numpy
import matplotlib.pyplot

frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')
backLegSensorValues = numpy.load('data/backLegSensorValues.npy')
frontLegMotorValues = numpy.load('data/frontLegMotorValues.npy')
backLegMotorValues = numpy.load('data/backLegMotorValues.npy')

# matplotlib.pyplot.plot(frontLegSensorValues, linewidth=3)
# matplotlib.pyplot.plot(backLegSensorValues)
matplotlib.pyplot.plot(frontLegMotorValues)
matplotlib.pyplot.plot(backLegMotorValues)


matplotlib.pyplot.legend()
matplotlib.pyplot.show()
# print(frontLegSensorValues)
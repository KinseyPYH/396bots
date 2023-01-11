import numpy
import matplotlib.pyplot

frontLegSensorValues = numpy.load('data/frontLegSensorValues.npy')

matplotlib.pyplot.plot(frontLegSensorValues)
matplotlib.pyplot.show()
print(frontLegSensorValues)
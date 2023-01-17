import constants as c
import numpy
import pyrosim.pyrosim as pyrosim
class SENSOR:
    def __init__(self, linkName):
        self.linkName = linkName
        self.values = numpy.zeros(c.numIterations)

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        # if (t >= c.numIterations-1):
        #     # print(self.values)
        #     self.Save_Values()

    def Save_Values(self):
        numpy.save("data/" + self.linkName + ".npy", self.values)

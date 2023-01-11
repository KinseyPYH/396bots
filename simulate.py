import pybullet_data
import pybullet as p
import pyrosim.pyrosim as pyrosim
import numpy

import time

t = 1/60
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
frontLegSensorValues = numpy.zeros(100)
# print("NUmpy values")
# print(backLegSensorValues)
# exit()


for i in range(100):
  p.stepSimulation()
  frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

  time.sleep(t)
  print(i)


p.disconnect()
print(frontLegSensorValues)
numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)

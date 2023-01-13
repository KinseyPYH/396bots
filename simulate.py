import pybullet_data
import pybullet as p
import pyrosim.pyrosim as pyrosim
import numpy

import random

import time

numIterations = 1000
t = 1/500
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

#front
f_amplitude = numpy.pi/4
f_frequency = 10
f_phaseOffset = numpy.pi/6
#back
b_amplitude = numpy.pi/4
b_frequency = 10
b_phaseOffset = 0

pyrosim.Prepare_To_Simulate(robotId)
frontLegSensorValues = numpy.zeros(numIterations)
backLegSensorValues = numpy.zeros(numIterations)
frontLegMotorValues = numpy.zeros(numIterations)
backLegMotorValues = numpy.zeros(numIterations)


x = numpy.linspace(0, 2*numpy.pi, numIterations)
# frontLegMotorValues = numpy.sin(x)
# frontLegMotorValues = frontLegMotorValues * numpy.pi/4
for i in range(numIterations):
  frontLegMotorValues[i] = f_amplitude * numpy.sin(f_frequency * x[i] + f_phaseOffset)
  backLegMotorValues[i] = b_amplitude * numpy.sin(b_frequency * x[i] + b_phaseOffset)


for i in range(numIterations):
  p.stepSimulation()
  frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
  backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
  
  pyrosim.Set_Motor_For_Joint( 
    bodyIndex = robotId,
    jointName = b'Torso_FrontLeg',
    controlMode = p.POSITION_CONTROL,
    targetPosition = frontLegMotorValues[i],
    maxForce = 300)

  pyrosim.Set_Motor_For_Joint( 
    bodyIndex = robotId,
    jointName = b'Torso_BackLeg',
    controlMode = p.POSITION_CONTROL,
    targetPosition = backLegMotorValues[i],
    maxForce = 300)


  time.sleep(t)
  print(i)


p.disconnect()
numpy.save("data/frontLegSensorValues.npy", frontLegSensorValues)
numpy.save("data/backLegSensorValues.npy", backLegSensorValues)
numpy.save("data/frontLegMotorValues.npy", frontLegMotorValues)
numpy.save("data/backLegMotorValues.npy", backLegMotorValues)


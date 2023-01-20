import pybullet_data
import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import constants as c
import numpy
from robot import ROBOT

class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF("body.urdf")
        p.loadSDF("world.sdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.robot = ROBOT()
        self.robot.Prepare_To_Sense()
        self.robot.Prepare_To_Act()

    def Run(self):
        
       
        for i in range(1000):
          p.stepSimulation()
          self.robot.Sense(i)
          self.robot.Think()
          self.robot.Act(self.robotId, i)
    
          time.sleep(c.t)
        #   print(i)

    def __del__(self):
          p.disconnect()

import pybullet_data
import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import constants as c
import numpy
from robot import ROBOT
import os

class SIMULATION:
    def __init__(self, directOrGUI, seednum, solutionID, brain_fileName, body_filename):
        print("Start simulation")
        self.directOrGUI = directOrGUI
        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        self.planeId = p.loadURDF("plane.urdf")
        p.loadSDF("world.sdf")
        self.solutionID = solutionID
        self.robot = ROBOT(seednum, solutionID, brain_fileName, body_filename)
        self.robot.Prepare_To_Sense()
        self.robot.Prepare_To_Act()

    def Run(self):
        for i in range(c.numIterations):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(self.robot, i)
            if self.directOrGUI == 'GUI':
                time.sleep(c.t)
        self.robot.Get_Fitness()
        
        #   print(i)

    # def Get_Fitness(self):
        # self.robot.Get_Fitness()


    def __del__(self):
          p.disconnect()

from simulation import SIMULATION
import sys
# import numpy
# import constants as c
# import random

# import time

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]

simulate = SIMULATION(directOrGUI, solutionID)
simulate.Run()
simulate.Get_Fitness()

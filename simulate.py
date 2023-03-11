from simulation import SIMULATION
import sys
# import numpy
# import constants as c
# import random

# import time

directOrGUI = sys.argv[1]
seednum = sys.argv[2]
solutionID = sys.argv[3]

simulate = SIMULATION(directOrGUI, int(seednum), solutionID, None, None)
simulate.Run()
# simulate.Get_Fitness()

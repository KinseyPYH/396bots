from simulation import SIMULATION
import sys
# import numpy
# import constants as c
# import random

# import time

directOrGUI = sys.argv[1]

simulate = SIMULATION(directOrGUI)
simulate.Run()
simulate.Get_Fitness()

import os
# from hillclimber import HILLCLIMBER
from parallelHillClimber import PARALLEL_HILLCLIMBER



phc = PARALLEL_HILLCLIMBER()
phc.Evolve()
phc.Show_Best()

# for i in range(5):
#     os.system("python3.7 generate.py")
#     os.system("python3.7 simulate.py")
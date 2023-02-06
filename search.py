import os
# from hillclimber import HILLCLIMBER
from parallelHillClimber import PARALLEL_HILLCLIMBER



phc = PARALLEL_HILLCLIMBER()
phc.Evolve()
phc.Show_Best()

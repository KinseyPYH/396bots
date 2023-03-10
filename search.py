import os
# from hillclimber import HILLCLIMBER
from parallelHillClimber import PARALLEL_HILLCLIMBER
import pickle
import constants as c

checkpoint_file = "phc_" + str(c.seed) + "_gen0" + ".pickle"

# phc.Evolve()
if os.path.exists(checkpoint_file):

    with open(checkpoint_file, 'rb') as f:
        phc = pickle.load(f)

else:
    phc = PARALLEL_HILLCLIMBER(checkpoint_file)

phc.Evolve()
phc.Show_Best()

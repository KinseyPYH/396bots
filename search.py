import os
import sys
# from hillclimber import HILLCLIMBER
from parallelHillClimber import PARALLEL_HILLCLIMBER
import pickle
import constants as c

checkpoint_file_bool = sys.argv[1]
seednum = sys.argv[2]
# print(c.seednum)
gennum = 0
if len(sys.argv) == 4:
    gennum = sys.argv[3] 
checkpoint_file = "seed" + str(seednum) + "/seed" + str(seednum) + "_gen" + str(gennum) + ".pickle"
loadcheckpoint = False

if str(checkpoint_file_bool) == "True" or str(checkpoint_file_bool) == "T" or str(checkpoint_file_bool) == "t":
    loadcheckpoint = True

elif str(checkpoint_file_bool) == "False" or str(checkpoint_file_bool) == "F" or str(checkpoint_file_bool) == "f":
    loadcheckpoint = False

else:
    print("Did not enter True/T/t or False/F/f for load pickle file option. Please enter correct parameters. ")
    exit()

if loadcheckpoint:
# os.path.exists(checkpoint_file):
    if not os.path.exists(checkpoint_file):
        print(checkpoint_file)
        print("No pickle file with this seed or generation. Please enter correct parameters")
        exit()
    with open(checkpoint_file, 'rb') as f:
        phc = pickle.load(f)

else:
    phc = PARALLEL_HILLCLIMBER(checkpoint_file, seednum)

phc.Evolve()
phc.Show_Best()

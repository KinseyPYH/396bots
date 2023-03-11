from simulation import SIMULATION
import sys

directory = sys.argv[1]
creatureNum = sys.argv[2]

brainfilename = directory + "/brain" + creatureNum + ".nndf" 
bodyfilename = directory + "/body" + creatureNum + ".urdf" 

# seed number does not matter for this. the directory will have the seednumber.
simulate = SIMULATION('GUI', 0, None, brainfilename, bodyfilename)
simulate.Run()

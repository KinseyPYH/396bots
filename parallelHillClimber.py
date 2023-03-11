import os
from solution import SOLUTION
import constants as c
import copy
import pickle
import numpy as np

class PARALLEL_HILLCLIMBER:
    def __init__(self, pickle_file, seednum):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        self.pickle_file = pickle_file
        self.seed = seednum #for pickle

        self.listBestFitnessSoFar = []
        self.listBestFitnessSoFarSolutions = []
        self.bestfitnessSoFar = float('inf')
        self.currentGeneration = 0

        self.parents = {}
        self.nextAvailableID = 0
        self.children = {}
        self.foldername =  "seed%s" %(self.seed)
        self.createPickle = True
        np.random.seed(int(seednum))

        filesToDelete = ['/brain*.nndf', '/fitness*.nndf', 'body*.urdf', '*.txt', '*.pickle']
        for file in filesToDelete:
            os.system("rm " + self.foldername + "/" + file)    
 

        
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID, seednum)
            self.nextAvailableID += 1


    def Evolve(self):
        self.Evaluate(self.parents)

        self.FindBestFit()

        while self.currentGeneration < c.numberOfGenerations:
            self.currentGeneration += 1
            self.Evolve_For_One_Generation()
            new_pickle_file = self.foldername + "/seed" + str(self.seed) + "_gen" + str(self.currentGeneration) + ".pickle"
            if self.createPickle:
                with open(new_pickle_file, 'wb') as f:
                    pickle.dump(self, f)
                self.createPickle = False
            

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Spawn(self):
        self.children = {}
        for parent in self.parents:
            self.children[parent] = copy.deepcopy(self.parents[parent])
            self.children[parent].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1


    def Mutate(self):
        for child in self.children:
            ch = self.children[child]
            ch.Mutate()
        # self.child.Mutate()


    def Select(self):
        
        for parent in self.parents:
            # work on saving robots of each best generation 

            ### if child is better than parent. 
            if (self.parents[parent].fitness > self.children[parent].fitness):
                ## If parent used to be a best in generation, keep. Else, delete.
                if (self.parents[parent].myID not in self.listBestFitnessSoFar):
                    bodyFileName = self.foldername + "/body" + str(self.parents[parent].myID) + ".urdf"
                    os.system("rm " + self.foldername + "/brain" + str(self.parents[parent].myID) + ".nndf")
                    os.system("rm " + bodyFileName)
                self.parents[parent] = self.children[parent]
            ## if child is worse, don't keep.
            else: 
                os.system("rm " + self.foldername + "/brain" + str(self.children[parent].myID) + ".nndf")
                bodyFileName = self.foldername + "/body" + str(self.children[parent].myID) + ".urdf"
                os.system("rm " + bodyFileName)
        
        # find best solution in generation and record fitness
        bestfit = self.FindBestFit()
    
        


    def Print(self):
        print("")
        for parent in self.parents:
            print("parent" + str(self.parents[parent].myID) + " fitness: " +  str(self.parents[parent].fitness) + ", children " + str(self.children[parent].myID) + " fitness: " + str(self.children[parent].fitness))
        print("")

    def Show_Best(self):
        bestFitness = float('inf')
        bestParent = None
        for parent in self.parents:
            if self.parents[parent].fitness < bestFitness:
                bestFitness = self.parents[parent].fitness
                bestParent = self.parents[parent]
        print("Best fitness-  "+ str(bestParent.myID) + ": " + str(bestFitness))
        bestParent.Start_Simulation('GUI')
        # self.child.Evaluate('GUI')
    
    def Evaluate(self, solutions):
        print("PHC Evaluate: Length of children: " + str(len(solutions)))

        for parent in solutions:
            solutions[parent].Start_Simulation('DIRECT')


        for parent in solutions:
            solutions[parent].Wait_For_Simulation_To_End()
            
    def FindBestFit(self):
        bestfit = float('inf')
        bestSolution = 0
        for parent in self.parents:
            if self.parents[parent].fitness < bestfit:
                bestfit = self.parents[parent].fitness
                bestSolution = self.parents[parent]
        
        # if best solution in generation is better than best solution so far, record this down. (keep pickled, too)
        if (bestfit < self.bestfitnessSoFar):
            self.bestfitnessSoFar = bestfit
            self.listBestFitnessSoFar.append(bestSolution.myID)
            f = open(self.foldername + "/increasesInFitness" + str(self.seed) + ".txt", "a")
            f.write("Fitness: " + str(self.bestfitnessSoFar) + " generation: " + str(self.currentGeneration) + " ID: " + str(bestSolution.myID) + '\n')
            f.close()
        f = open(self.foldername + "/BestFitnessEachGen_seed" + str(self.seed) + ".txt", "a")
        f.write(str(bestfit) + '\n')
        f.close()
        return bestfit
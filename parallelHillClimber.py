import os
from solution import SOLUTION
import constants as c
import copy
import pickle

class PARALLEL_HILLCLIMBER:
    def __init__(self, pickle_file):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        self.pickle_file = pickle_file
        self.seed = c.seed
        # os.system("rm allBestFitness0.txt")

        self.listBestFitnessSoFar = []
        self.listBestFitnessSoFarSolutions = []
        self.bestfitnessSoFar = float('inf')
        self.currentGeneration = 0

        self.parents = {}
        self.nextAvailableID = 0
        self.children = {}
        
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):
        self.Evaluate(self.parents)
        while self.currentGeneration < c.numberOfGenerations:
            self.Evolve_For_One_Generation()
            self.currentGeneration += 1
            

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
                    bodyFileName = "body" + str(self.parents[parent].myID) + ".urdf"
                    os.system("rm brain" + str(self.parents[parent].myID) + ".nndf")
                    os.system("rm " + bodyFileName)
                self.parents[parent] = self.children[parent]
            ## if child is worse, don't keep.
            else: 
                os.system("rm brain" + str(self.children[parent].myID) + ".nndf")
                bodyFileName = "body" + str(self.children[parent].myID) + ".urdf"
                # if (self.children[parent].myID not in self.listBestFitnessSoFar):
                os.system("rm " + bodyFileName)
        
        # find best solution in generation and record fitness
        bestfit = float('inf')
        bestSolution = 0
        for parent in self.parents:
            if self.parents[parent].fitness < bestfit:
                bestfit = self.parents[parent].fitness
                bestSolution = self.parents[parent]
        
        # if best solution in generation is better than best solution so far, record this down. (pickled, too)
        if (bestfit < self.bestfitnessSoFar):
            new_pickle_file = "saved_seed" + str(c.seed) + "_gen" + str(self.currentGeneration) + ".pickle"
            with open(new_pickle_file, 'wb') as f:
                pickle.dump(self, f)
            self.bestfitnessSoFar = bestfit
            self.listBestFitnessSoFar.append(bestSolution.myID)
            f = open("increasesInFitness" + str(c.seed) + ".txt", "a")
            f.write("Fitness: " + str(self.bestfitnessSoFar) + " generation: " + str(self.currentGeneration) + " ID: " + str(bestSolution.myID) + '\n')
            f.close()

        f = open("allBestFitness" + str(c.seed) + ".txt", "a")
        f.write(str(bestfit) + '\n')
        f.close()


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
            

import os
from solution import SOLUTION
import constants as c
import copy

class PARALLEL_HILLCLIMBER:
    def __init__(self):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")

        self.parents = {}
        self.nextAvailableID = 0

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

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
            self.children[child].Mutate()


    def Select(self):

        for parent in self.parents:
            calcFitness = self.parents[parent].fitness[0]*0.65 + self.parents[parent].fitness[1]*0.35
            childFitness = self.children[parent].fitness[0]*0.65 + self.children[parent].fitness[1]*0.35
            if childFitness > calcFitness:
                self.parents[parent] = self.children[parent]

    def Print(self):
        print("")
        
        for parent in self.parents:
            calcFitness = self.parents[parent].fitness[0]*0.65 + self.parents[parent].fitness[1]*0.35
            childFitness = self.children[parent].fitness[0]*0.65 + self.children[parent].fitness[1]*0.35
            print("parent fitness: " + str(calcFitness) + "->" + str(self.parents[parent].fitness) + ", children fitness: " + str(childFitness) + "->" + str(self.children[parent].fitness))
        print("")

    def Show_Best(self):
        bestFitness = self.parents[0].fitness[0]*0.65 + self.parents[0].fitness[1]*0.35
        bestParent = self.parents[0]
        for parent in self.parents:
            calcFitness = self.parents[parent].fitness[0]*0.65 + self.parents[parent].fitness[1]*0.35
            if calcFitness > bestFitness:
                bestFitness = calcFitness
                bestParent = self.parents[parent]
        bestParent.Start_Simulation('GUI')
        print("Best fitness: " + str(bestFitness))
    
    def Evaluate(self, solutions):
        for parent in solutions:
            # self.parents[parent].Evaluate('GUI')
            solutions[parent].Start_Simulation('DIRECT')
            # solutions[parent].Start_Simulation('GUI')

        # self.parent.Evaluate('GUI')

        for parent in solutions:
            solutions[parent].Wait_For_Simulation_To_End()
            
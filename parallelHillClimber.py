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
        # self.child.Evaluate('DIRECT')
        self.Print()
        self.Select()

    def Spawn(self):
        self.children = {}

        # self.nextAvailableID += 1
        for parent in self.parents:
            self.children[parent] = copy.deepcopy(self.parents[parent])
            self.children[parent].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1


    def Mutate(self):
        for child in self.children:
            self.children[child].Mutate()
        # self.child.Mutate()


    def Select(self):
        for parent in self.parents:
            # print("parent fitness: " +  str(self.parents[parent].fitness) + ", children fitness: " + str(self.children[parent].fitness))
            if (self.parents[parent].fitness > self.children[parent].fitness):
                self.parents[parent] = self.children[parent]


    def Print(self):
        print("")
        for parent in self.parents:
            print("parent fitness: " +  str(self.parents[parent].fitness) + ", children fitness: " + str(self.children[parent].fitness))
        print("")

    def Show_Best(self):
        bestFitness = float('inf')
        bestParent = None
        for parent in self.parents:
            if self.parents[parent].fitness < bestFitness:
                bestFitness = self.parents[parent].fitness
                bestParent = self.parents[parent]
        bestParent.Start_Simulation('GUI')
        print("Best fitness: " + str(bestFitness))
        # self.child.Evaluate('GUI')
    
    def Evaluate(self, solutions):
        for parent in solutions:
            # self.parents[parent].Evaluate('GUI')
            solutions[parent].Start_Simulation('DIRECT')
            # solutions[parent].Start_Simulation('GUI')

        # self.parent.Evaluate('GUI')

        for parent in solutions:
            solutions[parent].Wait_For_Simulation_To_End()
            
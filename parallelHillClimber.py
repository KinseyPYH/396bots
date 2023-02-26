import os
from solution import SOLUTION
import constants as c
import copy

class PARALLEL_HILLCLIMBER:
    def __init__(self):

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        # os.system("rm allBestFitness0.txt")

        self.parents = {}
        self.nextAvailableID = 0
        self.children = {}

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
            ch = self.children[child]
            ch.Mutate()
        # self.child.Mutate()


    def Select(self):
        for parent in self.parents:
            # print("parent fitness: " +  str(self.parents[parent].fitness) + ", children fitness: " + str(self.children[parent].fitness))
            if (self.parents[parent].fitness > self.children[parent].fitness):
                os.system("rm brain" + str(self.parents[parent].myID) + ".nndf")
                bodyFileName = "body" + str(self.parents[parent].myID) + ".urdf"
                os.system("rm " + bodyFileName)
                self.parents[parent] = self.children[parent]
            else:
                os.system("rm brain" + str(self.children[parent].myID) + ".nndf")
                bodyFileName = "body" + str(self.children[parent].myID) + ".urdf"
                os.system("rm " + bodyFileName)
        
        bestfit = float('inf')
        for parent in self.parents:
            if self.parents[parent].fitness < bestfit:
                bestfit = self.parents[parent].fitness
        f = open("allBestFitness2.txt", "a")
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
                print("FITNESS WTF: " + str(bestFitness))
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
            

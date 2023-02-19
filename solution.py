import os
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
import time
from operator import add

class SOLUTION:
    def __init__(self, nextAvailableID):
        # self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        # self.weights = (self.weights * 2) - 1 
        self.myID = nextAvailableID
        self.Create_World()
        self.Create_Body()

        
      
    def Evaluate(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        f.close()
        print(self.fitness)
        os.system("rm " + fitnessFileName)
        
    def Start_Simulation(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID))# + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        print(self.fitness)
        f.close()
        os.system("rm " + fitnessFileName)

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        # pyrosim.Send_Cube(name="Box", pos=[c.x-3,c.y+3,c.z] , size=[c.length, c.width, c.height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")


        directionsToGrow = ['up', 'down', 'left', 'right', 'front', 'back']
        directions = {
            'up':    [0, 0, 1 ],
            'down':  [0, 0, -1],
            'left':  [-1, 0, 0],
            'right': [1, 0, 0 ],
            'front': [0, 1, 0 ],
            'back':  [0, -1, 0],
            
        }

        jointDirections = ["0 1 0", "1 0 0"]

        parentJointDirections = {
            'up':    'down',
            'down':  'up',
            'left':  'right',
            'right': 'left',
            'front': 'back',
            'back':  'front',
        }

        self.totalNumLinks = np.random.randint(6, high=c.maxNumLinks)
        self.sensorColors = []
        for i in range(self.totalNumLinks):
            randomNum = np.random.rand()
            color = 'Blue'
            if randomNum > 0.5:
                color = 'Green'
            self.sensorColors.append(color)
        
        self.currentLinks = []
        self.allJoints = []
        length = np.random.rand() * c.maxSize + c.minLinkSize
        width = np.random.rand() * c.maxSize + c.minLinkSize
        height = np.random.rand() * c.maxSize + c.minLinkSize
        pyrosim.Send_Cube(name="0", pos=[0,0,height/2 + 2] , size=[length, width, height], color=self.sensorColors[0])
        linkObject = {
            "name": 0,
            "LWH": [length, width, height],
            "abs_pos": [0,0,height/2 + c.maxSize],
            "filled": [], #set of up, down, left, right, front, back, means which part of this link is filled
            "parentJointName": "", # empty means parent
            "parentJointDirection": "",
            "grewToward": None
        }
        randomDirectionToGrow = np.random.choice(directionsToGrow)
        randomDirectionToGrowCoord = directions[randomDirectionToGrow]
        jointPosition = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
        jointPosition[2] += height/2 + 2
        pyrosim.Send_Joint( name = "0_1" , parent= "0" , child = "1" , type = "revolute", position = jointPosition, jointAxis = np.random.choice(jointDirections))
        self.allJoints.append(("0_1", np.random.rand()))
        
        self.currentLinks.append(linkObject)

        randomParentLink = np.random.choice(self.currentLinks)
        length = np.random.rand() * c.maxSize + c.minLinkSize
        width = np.random.rand() * c.maxSize + c.minLinkSize
        height = np.random.rand() * c.maxSize + c.minLinkSize
        linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
        linkAbsPos = [a+b for a,b in zip(jointPosition, linkRelPos)]

        pyrosim.Send_Cube(name="1", pos=linkRelPos , size=[length, width, height], color=self.sensorColors[1])
        
        linkObject = {
            "name": 1,
            "LWH": [length, width, height],
            "abs_pos": linkAbsPos,
            "filled": [parentJointDirections[randomDirectionToGrow]], #set of up, down, left, right, front, back, means which part of this link is filled
            "parentJointName": "0_1", # empty means parent
            "parentJointDirection": parentJointDirections[randomDirectionToGrow],
            'grewToward': randomDirectionToGrow
        }

        ## also need to fill in parent object's filled
        self.currentLinks[0]['filled'].append(randomDirectionToGrow)

        ## add new object into links list
        self.currentLinks.append(linkObject)
        
        
        i = 2
        while i < self.totalNumLinks:

            randomParentLink = np.random.choice(self.currentLinks)
            randomParentLWH = randomParentLink['LWH']
            length, width, height = randomParentLWH[0], randomParentLWH[1], randomParentLWH[2]

            copyRandomDirectionsToGrow = directionsToGrow.copy()
            randomDirectionToGrow = np.random.choice(copyRandomDirectionsToGrow)
            copyRandomDirectionsToGrow.remove(randomDirectionToGrow)

            randomDirectionToGrowCoord = directions[randomDirectionToGrow]

            while len(copyRandomDirectionsToGrow) and randomDirectionToGrow in randomParentLink["filled"]:
                randomDirectionToGrow = np.random.choice(copyRandomDirectionsToGrow)
                copyRandomDirectionsToGrow.remove(randomDirectionToGrow)
                randomDirectionToGrowCoord = directions[randomDirectionToGrow]

            if randomDirectionToGrow in randomParentLink["filled"]:
                "Couldn't find direction to grow"
                continue
            
            ## For use to calculate new link absolute postiion
            jointRelPositionFromCenterOFParentLink = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]

            if randomParentLink['parentJointName'] == '':
                print("Passed: chose root")
                continue

            ## for use when Send_Joint()    
                # go from one edge (joint) of cube to another edge (joint) through the center of cube
            centerofParentLinkRelative = [float(a)*(b/2) for a,b in zip(directions[randomParentLink['grewToward']], randomParentLWH)]
            goToNewJointAdd= [(a/2)*float(b) for a,b in zip(randomParentLWH, directions[randomDirectionToGrow])]
            FromPrevJointToNewJointRelPosition = [a + b for a,b in zip(centerofParentLinkRelative, goToNewJointAdd)]


            length = np.random.rand() * c.maxSize + c.minLinkSize
            width = np.random.rand() * c.maxSize + c.minLinkSize
            height = np.random.rand() * c.maxSize + c.minLinkSize
            
            # When joint is added
            linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]

            ## Used to calculate collisions
            newJointCalculatedAbsPos = [a+b for a,b in zip(randomParentLink['abs_pos'], jointRelPositionFromCenterOFParentLink)]
            linkAbsPos = [a+b for a,b in zip(linkRelPos, newJointCalculatedAbsPos)]

            collisionCheckCount = 0 
            isCollide = self.Check_Collision(randomParentLink, linkAbsPos, length, width, height, randomDirectionToGrow)
            while (isCollide and collisionCheckCount < 11) or (linkAbsPos[2]-height/2) < 0:
                length = np.random.rand() * c.maxSize + c.minLinkSize
                width = np.random.rand() * c.maxSize + c.minLinkSize
                height = np.random.rand() * c.maxSize + c.minLinkSize
                
                linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
                linkAbsPos = [a+b for a,b in zip(linkRelPos, newJointCalculatedAbsPos)]
                collisionCheckCount += 1
                isCollide = self.Check_Collision(randomParentLink, linkAbsPos, length, width, height, randomDirectionToGrow)

            if collisionCheckCount > 10: #tried n times on this joint, doesnt work so try another one.
                print("Passed: this joint doesn't work")
                continue

            jointName = str(randomParentLink["name"]) + "_" + str(i)
            pyrosim.Send_Joint( name = jointName , parent= str(randomParentLink["name"]) , child = str(i) , type = "revolute", position =  FromPrevJointToNewJointRelPosition, jointAxis = np.random.choice(jointDirections))#"0 1 0")
            self.allJoints.append((jointName, np.random.rand()))
            pyrosim.Send_Cube(name=str(i), pos=linkRelPos , size=[length, width, height], color=self.sensorColors[i])

            linkObject = {
                "name": i,
                "LWH": [length, width, height],
                "abs_pos": linkAbsPos,
                "filled": [parentJointDirections[randomDirectionToGrow]], #set of up, down, left, right, front, back, means which part of this link is filled
                "parentJointName": randomParentLink['name'],
                "parentJointDirection": parentJointDirections[randomDirectionToGrow],
                "grewToward": randomDirectionToGrow
            }

            self.currentLinks[randomParentLink['name']]['filled'].append(randomDirectionToGrow)

            self.currentLinks.append(linkObject)
            
            i+=1
        pyrosim.End()


    def Check_Collision(self, parentLink, linkAbsPos, length, width, height, randomDirectionToGrow):
        for otherLink in self.currentLinks:
            if otherLink == parentLink:
                continue
            otherLinkAbsPos = otherLink['abs_pos']
            otherLinkLWH = otherLink['LWH']
            if (((abs(otherLinkAbsPos[0]-linkAbsPos[0]) - (otherLinkLWH[0]/2 + length/2)) < 0) and 
                ((abs(otherLinkAbsPos[1]-linkAbsPos[1]) - (otherLinkLWH[1]/2 + width/2 )) < 0) and
                ((abs(otherLinkAbsPos[2]-linkAbsPos[2]) - (otherLinkLWH[2]/2 + height/2)) < 0)):
                print("----in collide:---- ")
                # print(randomDirectionToGrow)
                # print(len(self.currentLinks))
                # print(self.currentLinks)
                # print("parent: " + str(parentLink['name']))
                # print(length,width,height)
                # print("       --- conditions ---")
                # print((abs(otherLinkAbsPos[0]-linkAbsPos[0]) - (otherLinkLWH[0]/2 + length/2)))
                # print((abs(otherLinkAbsPos[1]-linkAbsPos[1]) - (otherLinkLWH[1]/2 + width/2 )))
                # print((abs(otherLinkAbsPos[2]-linkAbsPos[2]) - (otherLinkLWH[2]/2 + height/2)))
                # print("       --- end condition ---")

                # print(otherLinkAbsPos)
                # print(linkAbsPos)
                # print(otherLinkLWH)
                # print([length/2, width/2, height/2])
                # print("-----end colide -----")
                return True
        return False

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        sensorNeuronCount = 0
        for i in range(len(self.currentLinks)):
            if self.sensorColors[i] == 'Green':
                pyrosim.Send_Sensor_Neuron(name = sensorNeuronCount, linkName = str(i))
                sensorNeuronCount += 1 
            if sensorNeuronCount == 0 and i == (len(self.currentLinks) - 1):
                pyrosim.Send_Sensor_Neuron(name = sensorNeuronCount, linkName = str(i))
                sensorNeuronCount += 1 


        c.numSensorNeurons = sensorNeuronCount
        motorNeuronCount = 0
        for i in range(len(self.allJoints)):
            if self.allJoints[i][1] > 0.5:
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.allJoints[i][0])
                motorNeuronCount += 1
            if motorNeuronCount == 0 and i == (len(self.currentLinks) - 1):
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.allJoints[i][0])
                motorNeuronCount += 1

            
        c.numMotorNeurons = motorNeuronCount
        print("Motors: " + str(c.numMotorNeurons))
        self.weights = np.random.rand(sensorNeuronCount, motorNeuronCount)

        self.weights = (self.weights * 2) - 1 
        for currentRow in range(sensorNeuronCount):
            for currentColumn in range(motorNeuronCount):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn + sensorNeuronCount , weight = 1)#self.weights[currentRow][currentColumn])
                
        pyrosim.End()
    

    def Mutate(self):
        # print(c.numSensorNeurons-1)
        numSensorNeurons = c.numSensorNeurons - 1
        numMotorNeurons = c.numMotorNeurons - 1

        if (c.numSensorNeurons == 1):
            numSensorNeurons = 0
        if (c.numMotorNeurons == 1):
            numMotorNeurons = 0
        randomRow = random.randint(0, numSensorNeurons)
        randomColumn = random.randint(0,numMotorNeurons)
        self.weights[randomRow, randomColumn] = (np.random.rand() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID

    
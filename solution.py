import os
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as c
import time
from operator import add

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

class SOLUTION:
    def __init__(self, nextAvailableID):
        # self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        # self.weights = (self.weights * 2) - 1 
        self.myID = nextAvailableID
        self.Create_World()
        # self.Create_Body()

        
      
    def Evaluate(self, directOrGUI):
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID) + " 2&>1 &")
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm " + fitnessFileName)
        
    def Start_Simulation(self, directOrGUI):
        self.Create_Body()
        self.Create_Brain()
        os.system('python3.7 simulate.py ' + directOrGUI + " " + str(self.myID))# + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        f = open(fitnessFileName, "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm " + fitnessFileName)

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        # pyrosim.Send_Cube(name="Box", pos=[c.x-3,c.y+3,c.z] , size=[c.length, c.width, c.height])
        pyrosim.End()

    def Create_Body(self):


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
        # pyrosim.Send_Cube(name="0", pos=[0,0,height/2 + c.startHeight] , size=[length, width, height], color=self.sensorColors[0])
        linkObject = {
            "name": 0,
            "LWH": [length, width, height],
            "abs_pos": [0,0,height/2 + c.startHeight],
            "filled": [], #set of up, down, left, right, front, back, means which part of this link is filled
            "parentJointName": "", # empty means parent
            "parentJointDirection": "",
            "grewToward": None,
            "children": [],
            "RelPosFromParent": []
        }
        randomDirectionToGrow = np.random.choice(directionsToGrow)
        randomDirectionToGrowCoord = directions[randomDirectionToGrow]
        jointPosition = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
        jointPosition[2] += height/2 + c.startHeight
        randomJointAxis = np.random.choice(jointDirections)
        # pyrosim.Send_Joint( name = "0_1" , parent= "0" , child = "1" , type = "revolute", position = jointPosition, jointAxis = randomJointAxis)
        jointObject = {
            "name": "0_1",
            "jointPos": jointPosition, #first one is absolute, others are relative
            "chance": np.random.rand(),
            "jointAxis": randomJointAxis,
            "parentName": "0",
            "childName": "1"
        }
        # self.allJoints.append(("0_1", jointPosition, np.random.rand(), randomJointAxis)) # name, relposfromparent, probability, jointAxis
        self.allJoints.append(jointObject)
        self.currentLinks.append(linkObject)

        randomParentLink = np.random.choice(self.currentLinks)
        length = np.random.rand() * c.maxSize + c.minLinkSize
        width = np.random.rand() * c.maxSize + c.minLinkSize
        height = np.random.rand() * c.maxSize + c.minLinkSize
        linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
        linkAbsPos = [a+b for a,b in zip(jointPosition, linkRelPos)]

        # pyrosim.Send_Cube(name="1", pos=linkRelPos , size=[length, width, height], color=self.sensorColors[1])
        
        linkObject = {
            "name": 1,
            "LWH": [length, width, height],
            "abs_pos": linkAbsPos,
            "filled": [parentJointDirections[randomDirectionToGrow]], #set of up, down, left, right, front, back, means which part of this link is filled
            "parentJointName": "0_1", # empty means parent
            "parentJointDirection": parentJointDirections[randomDirectionToGrow],
            'grewToward': randomDirectionToGrow,
            "children": [],
            "RelPosFromParent": linkRelPos
        }

        ## also need to fill in parent object's filled
        self.currentLinks[0]['filled'].append(randomDirectionToGrow)
        self.currentLinks[0]['children'].append(linkObject)
        ## add new object into links list
        self.currentLinks.append(linkObject)
        
        
        i = 2
        while i < self.totalNumLinks:
            print("Loop")
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
                print("Couldn't find direction to grow")
                continue
            
            ## For use to calculate new link absolute postiion
            jointRelPositionFromCenterOFParentLink = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]

            if randomParentLink['parentJointName'] == '':
                print("Root")
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
            while (isCollide and collisionCheckCount < 11) or (linkAbsPos[2]-height/2 and collisionCheckCount < 11) < 0:
                length = np.random.rand() * c.maxSize + c.minLinkSize
                width = np.random.rand() * c.maxSize + c.minLinkSize
                height = np.random.rand() * c.maxSize + c.minLinkSize
                
                linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
                # print("linkabspos")
                linkAbsPos = [a+b for a,b in zip(linkRelPos, newJointCalculatedAbsPos)]
                collisionCheckCount += 1
                isCollide = self.Check_Collision(randomParentLink, linkAbsPos, length, width, height, randomDirectionToGrow)

            if collisionCheckCount > 10: #tried n times on this joint, doesnt work so try another one.
                print("Collision Check Count > 10")
                continue

            jointName = str(randomParentLink["name"]) + "_" + str(i)
            randomJointAxis = np.random.choice(jointDirections)
            # pyrosim.Send_Joint( name = jointName , parent= str(randomParentLink["name"]) , child = str(i) , type = "revolute", position =  FromPrevJointToNewJointRelPosition, jointAxis = randomJointAxis)#"0 1 0")
            jointObject = {
                "name": jointName,
                "jointPos": FromPrevJointToNewJointRelPosition, #first one is absolute, others are relative
                "chance": np.random.rand(),
                "jointAxis": randomJointAxis,
                "parentName": str(randomParentLink["name"]),
                "childName": str(i)
            }
            self.allJoints.append(jointObject)

            # self.allJoints.append((jointName, FromPrevJointToNewJointRelPosition, np.random.rand(), randomJointAxis, str(randomParentLink["name"]), str(i)))
            # pyrosim.Send_Cube(name=str(i), pos=linkRelPos , size=[length, width, height], color=self.sensorColors[i])

            linkObject = {
                "name": i,
                "LWH": [length, width, height],
                "abs_pos": linkAbsPos,
                "filled": [parentJointDirections[randomDirectionToGrow]], #set of up, down, left, right, front, back, means which part of this link is filled
                "parentJointName": jointName, #randomParentLink['name'],
                "parentJointDirection": parentJointDirections[randomDirectionToGrow],
                "grewToward": randomDirectionToGrow,
                "children": [],
                "RelPosFromParent": linkRelPos
            }
            
            self.currentLinks[randomParentLink['name']]['filled'].append(randomDirectionToGrow)
            self.currentLinks[randomParentLink['name']]['children'].append(linkObject)
            self.currentLinks.append(linkObject)
            
            i+=1

        self.Rebuild_Body()

    def Check_Collision(self, parentLink, linkAbsPos, length, width, height, randomDirectionToGrow):
        for otherLink in self.currentLinks:
            if otherLink == parentLink:
                continue
            otherLinkAbsPos = otherLink['abs_pos']
            otherLinkLWH = otherLink['LWH']
            if (((abs(otherLinkAbsPos[0]-linkAbsPos[0]) - (otherLinkLWH[0]/2 + length/2)) < 0) and 
                ((abs(otherLinkAbsPos[1]-linkAbsPos[1]) - (otherLinkLWH[1]/2 + width/2 )) < 0) and
                ((abs(otherLinkAbsPos[2]-linkAbsPos[2]) - (otherLinkLWH[2]/2 + height/2)) < 0)):
                # print("-----begin colide -----")
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
                # print([length, width, height])
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


        self.numSensorNeurons = sensorNeuronCount
        motorNeuronCount = 0
        for i in range(len(self.allJoints)):
            if self.allJoints[i]["chance"] > 0.5:
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.allJoints[i]["name"])
                motorNeuronCount += 1
            if motorNeuronCount == 0 and i == (len(self.allJoints) - 1):
                pyrosim.Send_Motor_Neuron( name = sensorNeuronCount + motorNeuronCount , jointName =  self.allJoints[i]["name"])
                motorNeuronCount += 1

            
        self.numMotorNeurons = motorNeuronCount
        self.weights = np.random.rand(sensorNeuronCount, motorNeuronCount)

        # self.weights = (self.weights * 2) - 1 
        for currentRow in range(sensorNeuronCount):
            for currentColumn in range(motorNeuronCount):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn + sensorNeuronCount , weight = self.weights[currentRow][currentColumn])
                
        pyrosim.End()
    
    def Rebuild_Body(self):
        pyrosim.Start_URDF("body.urdf")

        # U have all relative positions from parent. Simply run through that and build it.  
        pyrosim.Send_Cube(name="0", pos=self.currentLinks[0]["abs_pos"], size=self.currentLinks[0]["LWH"], color=self.sensorColors[0])
        pyrosim.Send_Joint( name = "0_1" , parent= "0" , child = "1" , type = "revolute", position = self.allJoints[0]["jointPos"], jointAxis = self.allJoints[0]["jointAxis"])
        pyrosim.Send_Cube(name="1", pos=self.currentLinks[1]["RelPosFromParent"], size=self.currentLinks[1]["LWH"], color=self.sensorColors[1])
        for i in range(2,len(self.currentLinks)):
            currLink = self.currentLinks[i]
            nextJoint = self.allJoints[i-1]
            pyrosim.Send_Joint(name=str(nextJoint["name"]), parent=nextJoint["parentName"], child=nextJoint["childName"], type="revolute", position = nextJoint["jointPos"], jointAxis=nextJoint["jointAxis"])
            pyrosim.Send_Cube(name=str(currLink['name']), pos=currLink['RelPosFromParent'], size=currLink['LWH'], color=self.sensorColors[i])
        pyrosim.End()
        
        # pass

        


    def Mutate(self):
        print("MUTATING....")
        # Three choices for mutation: 
        # 1. Change weights
        # 2. Change link
            # 1. Choose random block
            # 2. Delete all its children and all its children (and their joints)
            # 3. Regrow at that joint.
        # 3. Change n number of joints axes to the other joint axis. 
        self.MutateLinks()
        pass

    def MutateLinks(self):
        # randomLinkIndex = np.random.randint(0,high=len(self.currentLinks))
        print(self.currentLinks)
        deletedLinkNames = []
        toDeleteLinks = []
        childrenLinks = [] #for BFS
        randomLink = np.random.choice(self.currentLinks)
        # while randomLink["name"] == 0: #can't be parent...
        #     randomLink = np.random.choice(self.currentLinks)

        print("RANDOM LINK: " + str(randomLink))
        childrenLinks.append(randomLink)
        # toDeleteLinks.append(randomLink)
        while len(childrenLinks):
            childLink = childrenLinks[0]
            toDeleteLinks.append(childLink)
            deletedLinkNames.append(childLink["name"])
            for link in childLink["children"]:
                # childLink["children"].remove(link)
                for child in self.currentLinks[["children"]:
                    if child["name"] == link["name"]:
                        curr
                childrenLinks.append(link)
            # randomLink = childrenLinks[0]
            childrenLinks.pop(0)

        print("TO DELETE LINKS: ")
        print(toDeleteLinks)
        print("NAMES OF DELETED LINKS")
        print(deletedLinkNames)
        print("END MUTATE")

        for link in reversed(toDeleteLinks):
            self.currentLinks.remove(link)

        print("NEw links")
        print(self.currentLinks)


        # while i < self.totalNumLinks:
        #     print("Loop")
        #     randomParentLink = np.random.choice(self.currentLinks)
        #     randomParentLWH = randomParentLink['LWH']
        #     length, width, height = randomParentLWH[0], randomParentLWH[1], randomParentLWH[2]

        #     copyRandomDirectionsToGrow = directionsToGrow.copy()
        #     randomDirectionToGrow = np.random.choice(copyRandomDirectionsToGrow)
        #     copyRandomDirectionsToGrow.remove(randomDirectionToGrow)

        #     randomDirectionToGrowCoord = directions[randomDirectionToGrow]

        #     while len(copyRandomDirectionsToGrow) and randomDirectionToGrow in randomParentLink["filled"]:
        #         randomDirectionToGrow = np.random.choice(copyRandomDirectionsToGrow)
        #         copyRandomDirectionsToGrow.remove(randomDirectionToGrow)
        #         randomDirectionToGrowCoord = directions[randomDirectionToGrow]

        #     if randomDirectionToGrow in randomParentLink["filled"]:
        #         print("Couldn't find direction to grow")
        #         continue
            
        #     ## For use to calculate new link absolute postiion
        #     jointRelPositionFromCenterOFParentLink = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]

        #     if randomParentLink['parentJointName'] == '':
        #         print("Root")
        #         continue

        #     ## for use when Send_Joint()    
        #         # go from one edge (joint) of cube to another edge (joint) through the center of cube
        #     centerofParentLinkRelative = [float(a)*(b/2) for a,b in zip(directions[randomParentLink['grewToward']], randomParentLWH)]
        #     goToNewJointAdd= [(a/2)*float(b) for a,b in zip(randomParentLWH, directions[randomDirectionToGrow])]
        #     FromPrevJointToNewJointRelPosition = [a + b for a,b in zip(centerofParentLinkRelative, goToNewJointAdd)]


        #     length = np.random.rand() * c.maxSize + c.minLinkSize
        #     width = np.random.rand() * c.maxSize + c.minLinkSize
        #     height = np.random.rand() * c.maxSize + c.minLinkSize
            
        #     # When joint is added
        #     linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]

        #     ## Used to calculate collisions
        #     newJointCalculatedAbsPos = [a+b for a,b in zip(randomParentLink['abs_pos'], jointRelPositionFromCenterOFParentLink)]
        #     linkAbsPos = [a+b for a,b in zip(linkRelPos, newJointCalculatedAbsPos)]

        #     collisionCheckCount = 0 
        #     isCollide = self.Check_Collision(randomParentLink, linkAbsPos, length, width, height, randomDirectionToGrow)
        #     while (isCollide and collisionCheckCount < 11) or (linkAbsPos[2]-height/2 and collisionCheckCount < 11) < 0:
        #         length = np.random.rand() * c.maxSize + c.minLinkSize
        #         width = np.random.rand() * c.maxSize + c.minLinkSize
        #         height = np.random.rand() * c.maxSize + c.minLinkSize
                
        #         linkRelPos = [a*b for a,b in zip(randomDirectionToGrowCoord, [length/2, width/2, height/2])]
        #         # print("linkabspos")
        #         linkAbsPos = [a+b for a,b in zip(linkRelPos, newJointCalculatedAbsPos)]
        #         collisionCheckCount += 1
        #         isCollide = self.Check_Collision(randomParentLink, linkAbsPos, length, width, height, randomDirectionToGrow)

        #     if collisionCheckCount > 10: #tried n times on this joint, doesnt work so try another one.
        #         print("Collision Check Count > 10")
        #         continue

        #     jointName = str(randomParentLink["name"]) + "_" + str(i)
        #     randomJointAxis = np.random.choice(jointDirections)
        #     # pyrosim.Send_Joint( name = jointName , parent= str(randomParentLink["name"]) , child = str(i) , type = "revolute", position =  FromPrevJointToNewJointRelPosition, jointAxis = randomJointAxis)#"0 1 0")
        #     jointObject = {
        #         "name": jointName,
        #         "jointPos": FromPrevJointToNewJointRelPosition, #first one is absolute, others are relative
        #         "chance": np.random.rand(),
        #         "jointAxis": randomJointAxis,
        #         "parentName": str(randomParentLink["name"]),
        #         "childName": str(i)
        #     }
        #     self.allJoints.append(jointObject)

        #     # self.allJoints.append((jointName, FromPrevJointToNewJointRelPosition, np.random.rand(), randomJointAxis, str(randomParentLink["name"]), str(i)))
        #     # pyrosim.Send_Cube(name=str(i), pos=linkRelPos , size=[length, width, height], color=self.sensorColors[i])

        #     linkObject = {
        #         "name": i,
        #         "LWH": [length, width, height],
        #         "abs_pos": linkAbsPos,
        #         "filled": [parentJointDirections[randomDirectionToGrow]], #set of up, down, left, right, front, back, means which part of this link is filled
        #         "parentJointName": jointName, #randomParentLink['name'],
        #         "parentJointDirection": parentJointDirections[randomDirectionToGrow],
        #         "grewToward": randomDirectionToGrow,
        #         "children": [],
        #         "RelPosFromParent": linkRelPos
        #     }
            
        #     self.currentLinks[randomParentLink['name']]['filled'].append(randomDirectionToGrow)
        #     self.currentLinks[randomParentLink['name']]['children'].append(linkObject)
        #     self.currentLinks.append(linkObject)





    def RandomiseJointAxes(self):
        # jointDirections = ["0 1 0", "1 0 0"]
        for i in range(0, np.random.randint(1, high=len(self.allJoints))):
            self.allJoints[i][3] = np.random.choice(jointDirections)


    def ChangeWeights(self):
        numSensorNeurons =  self.numSensorNeurons - 1
        numMotorNeurons = self.numMotorNeurons - 1
        # print(numSensorNeurons)
        # print(numMotorNeurons)
        if (self.numSensorNeurons == 1):
            numSensorNeurons = 1
        if (self.numMotorNeurons == 1):
            numMotorNeurons = 1

        for i in range(3):
            randomRow = np.random.randint(0, high=numSensorNeurons)
            randomColumn = np.random.randint(0,high=numMotorNeurons)
        # print(randomRow)
        # print(randomColumn)
        # print(self.weights)
        # print(self.weights[0][0])
            self.weights[randomRow][randomColumn] = (np.random.rand() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID

    
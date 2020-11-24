
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from tree_search import *
from Challenge3 import *
from Dirs import *
import math

CELLROWS=7
CELLCOLS=14

def getReversePath(actions):
    reverse = []
    for a in reversed(actions):
        reverse.append(opposite(a))

    return reverse

def opposite(action):
    if action == Dirs.NORTH:
        return Dirs.SOUTH
    elif action == Dirs.EAST:
        return Dirs.WEST
    elif action == Dirs.SOUTH:
        return Dirs.NORTH
    else:   
        return Dirs.EAST

def result(pos, action):
    if action == Dirs.NORTH:
        return (pos[0], pos[1]+2)
    elif action == Dirs.EAST:
        return (pos[0]+2, pos[1])
    elif action == Dirs.SOUTH:
        return (pos[0], pos[1]-2)
    else:   
        return (pos[0]-2, pos[1])

class Node:
    def __init__(self, state, parent, actions):
        self.state = state
        self.parent = parent
        self.actions = actions

    def getAction(self):
        if len(self.actions) > 0: return self.actions.pop(0)
        else: return None

    def inParent(self, state):
        if not self.parent:
            return False
        if self.parent.state == state:
            return True
        return self.parent.inParent(state)


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.first = True
        # robot representation
        self.x = 0.0
        self.y = 0.0
        #self.targetX = None
        #self.targetY = None
        self.ang = 0
        self.outL = 0
        self.outR = 0
        self.state = "explore"
        # movement
        self.result = [self.x,self.y]
        self.action = Dirs.EAST
        self.visitedNodes = {}
        self.currentNode = None
        # return
        self.goBack = False
        self.returnActions = []
        # walls
        self.walls = []

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        first = True
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        while True:
            self.readSensors()
            self.ang = self.measures.compass
            self.fixPosition()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()
            # we start here
            if state == 'stop' and self.measures.start:
                if first:
                    first = False
                    actions = self.getActions((0,0))
                    self.currentNode = Node((0,0), None, actions)
                    # add node to set of visited nodes for when we need to return to it
                    self.visitedNodes[(0, 0)] = self.currentNode
                    print("//////////////////////////////////")
                    print("First Node")
                    print("Creating Node {} {}".format(0, 0))
                    print("Node actions {}".format(actions))
                    print("//////////////////////////////////")
                    previous_action = self.action
                    self.action = self.currentNode.getAction()
                    if previous_action != self.action:
                        self.state = "rotate"
                    self.updateResult()
                    # severe error if no actions are available at first instant (robot is placed in a locked 'cell' :) )
                    if not self.action:
                        print("Robot placed in locked cell. Quiting")
                        quit() 
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                # At Beacon
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                    # start going back
                    if not self.goBack:
                        self.goBack = True
                        roundX = int(round( self.x/2 ))*2
                        roundY = int(round( self.y/2 ))*2
                        d = Challenge3(self.visitedNodes, self.walls)
                        p = SearchProblem(d, (roundX, roundY), (0, 0))
                        t = SearchTree(p)
                        path, _, _ = t.search()
                        print(path)
                        self.returnActions = path[1:]
                        self.returnActions.append(Dirs.FINISH)
                        action = self.action
                        self.action = self.returnActions.pop(0)
                        if action != self.action:
                            self.state = "rotate"
                        self.updateResult()
                        # action = self.action
                        # nNode = Node((roundX, roundY), self.currentNode, [])
                        # self.currentNode = nNode
                        # self.visitedNodes[(roundX, roundY)] = self.currentNode
                        # print("//////////////////////////////////")
                        # print("Creating Node {} {} with parent {} {}".format(roundX, roundY, self.currentNode.parent.state[0], self.currentNode.parent.state[1]))
                        # print("Node actions {}".format([]))
                        # print("//////////////////////////////////")
                        # self.action = self.actionToParent()
                        # if action != self.action:
                        #     self.state = "rotate"
                        # self.updateResult()
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        center = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        collision = self.measures.collision

        lPow, rPow = 0.0 , 0.0
        # TODO move this to init
        # if self.first:
        #     #search path
        #     d = Challenge1(self.labMap)
        #     p = SearchProblem(d, [self.x, self.y], [self.targetX, self.targetY])
        #     t = SearchTree(p)
        #     path, _, _ = t.search() 
        #     self.actions = path[1:] #ignore first action which is none
        #     #add reverse path to go back
        #     self.actions += getReversePath(self.actions)
        #     self.actions.append(Dirs.STOP)
        #     self.updateResult()
        #     self.first = False
        #     if self.actions[0] != Dirs.EAST: self.state = "rotate"   # robot starts oriented towards east so rotate if first actions is not east
        lPow, rPow = 0.0 , 0.0
        if self.action == Dirs.FINISH:
            self.driveMotors(lPow, rPow)
            self.state = "finish"
            return
        self.setNextAction()
        # print("---------------------------------------")
        # print("Action-> {}".format(self.action))
        # print("Rotating -> {} {}".format(self.state == "rotate", self.angVariance(self.targetRotation())))
        # print("Think -> {},{}".format(self.x, self.y))
        # print("Going -> {},{}".format(self.result[0], self.result[1]))
        # print("---------------------------------------")
        targetRotation = self.targetRotation()

        if self.state == "rotate":
            dir_ = self.angVariance(self.targetRotation())
            if abs(dir_)<=5:
                
                self.state = 'explore'
                # if self.measures.ground==0:
                #     lPow = -self.outL
                #     rPow = -self.outR
                # else:        
                lPow = min(0.15, 0.14-self.outL) #stop robot giving it the oposite direction
                rPow = min(0.15, 0.14-self.outR) #stop robot giving it the oposite direction

            else:
                power = min(0.15, abs(dir_)/275) #rotate faster the higher the angle 

                lPow = -power * [-1,1][dir_<0] 
                rPow = power * [-1,1][dir_<0]
        else:
            adjust = self.rotationAdjustment(targetRotation)
            adjust = self.translationAdjustment(left, right, adjust)
            if center > 5 :
                lPow = -0.1 + adjust
                rPow = -0.1 - adjust
            else :
                lPow = 0.1 + adjust
                rPow = 0.1 - adjust
            
            if self.action == Dirs.STOP or self.action == Dirs.FINISH:
                lPow = 0.0
                rPow = 0.0


        self.updatePosition(lPow, rPow)
        self.driveMotors(lPow, rPow)

    def getActions(self, pos):
        front = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        print("front ",front)
        print("left",left)
        print("right",right)
        print("back",back)

        actions = [Dirs.SOUTH, Dirs.EAST, Dirs.WEST, Dirs.NORTH]
        # check wall on each side
        if self.action == Dirs.NORTH:
            if front > 1.5: 
                self.walls.append([pos, Dirs.NORTH] )
                actions.remove(Dirs.NORTH)
            if right > 1.5: 
                self.walls.append([pos, Dirs.EAST] )
                actions.remove(Dirs.EAST)
            if back > 1.5: 
                self.walls.append([pos, Dirs.SOUTH] )
                actions.remove(Dirs.SOUTH)
            if left > 1.5: 
                self.walls.append([pos, Dirs.WEST] )
                actions.remove(Dirs.WEST)
        elif self.action == Dirs.EAST:
            if front > 1.5: 
                self.walls.append([pos, Dirs.EAST] )
                actions.remove(Dirs.EAST)
            if right > 1.5: 
                self.walls.append([pos, Dirs.SOUTH] )
                actions.remove(Dirs.SOUTH)
            if back > 1.5: 
                self.walls.append([pos, Dirs.WEST] )
                actions.remove(Dirs.WEST)
            if left > 1.5: 
                self.walls.append([pos, Dirs.NORTH] )
                actions.remove(Dirs.NORTH)
        elif self.action == Dirs.SOUTH:
            if front > 1.5: 
                self.walls.append([pos, Dirs.SOUTH] )
                actions.remove(Dirs.SOUTH)
            if right > 1.5: 
                self.walls.append([pos, Dirs.WEST] )
                actions.remove(Dirs.WEST)
            if back > 1.5: 
                self.walls.append([pos, Dirs.NORTH] )
                actions.remove(Dirs.NORTH)
            if left > 1.5: 
                self.walls.append([pos, Dirs.EAST] )
                actions.remove(Dirs.EAST)
        elif self.action == Dirs.WEST:
            if front > 1.5: 
                self.walls.append([pos, Dirs.WEST] )
                actions.remove(Dirs.WEST)
            if right > 1.5: 
                self.walls.append([pos, Dirs.NORTH] )
                actions.remove(Dirs.NORTH)
            if back > 1.5: 
                self.walls.append([pos, Dirs.EAST] )
                actions.remove(Dirs.EAST)
            if left > 1.5: 
                self.walls.append([pos, Dirs.SOUTH] )
                actions.remove(Dirs.SOUTH)
        elif self.action == FINISH:
            return []
        return actions

    def actionToParent(self):
        parent = self.currentNode.parent
        node = self.currentNode

        if parent.state[1] > node.state[1]:
            return Dirs.NORTH
        elif parent.state[0] > node.state[0]:
            return Dirs.EAST
        elif parent.state[1] < node.state[1]:
            return Dirs.SOUTH
        elif parent.state[0] < node.state[0]:
            return Dirs.WEST
    
    def updateResult(self):
        if self.action == Dirs.NORTH:
            self.result[1] += 2
        elif self.action == Dirs.EAST :
            self.result[0] += 2
        elif self.action == Dirs.SOUTH :
            self.result[1] -= 2
        elif self.action == Dirs.WEST :
            self.result[0] -= 2
        elif self.action == Dirs.FINISH: 
            self.result = [0,0]

    def nextAction(self, goBack=False):
        roundX = int(round( self.x/2 ))*2
        roundY = int(round( self.y/2 ))*2
        if goBack:
            # self.currentNode = self.currentNode.parent
            # if not self.currentNode.parent:
            #     return Dirs.FINISH
            # print("//////////////////////////////////")
            # print("Back at Node {} {} with parent".format(roundX, roundY))
            # print("Node actions {}".format(self.currentNode.actions))
            # print("//////////////////////////////////")
            # return self.actionToParent()
            return self.returnActions.pop(0) 
        #if this cell has been visited
        if (roundX, roundY) in self.visitedNodes.keys():
            #bactrack
            self.currentNode = self.visitedNodes[(roundX, roundY)]
            print("//////////////////////////////////")
            print("Back at Node {} {}".format(roundX, roundY))
            print("Node actions {}".format(self.currentNode.actions))
            print("//////////////////////////////////")
            if len(self.currentNode.actions) == 0:
                # if no action we walk back to parent (maybe he'll have some)
                return self.actionToParent()
            else:
                return self.currentNode.getAction()
        else:
            #create new node and set action as the first in the new node action list
            nodeActions = self.getActions((roundX, roundY))
            nNode = Node((roundX, roundY), self.currentNode, nodeActions)
            self.currentNode = nNode
            #remove parent actions
            self.currentNode.actions = [a for a in self.currentNode.actions if not self.currentNode.inParent(result(self.currentNode.state,a))]
            self.visitedNodes[(roundX, roundY)] = self.currentNode
            print("//////////////////////////////////")
            if self.currentNode.parent != None:
                print("Creating Node {} {} with parent {} {}".format(roundX, roundY, self.currentNode.parent.state[0], self.currentNode.parent.state[1]))
            else:
                print("Creating Node {} {}".format(roundX, roundY))
            print("Node actions {}".format(self.currentNode.actions))
            print("//////////////////////////////////")
            # we're at a dead end walk back
            if len(self.currentNode.actions) == 0: # walk back to parent
                return self.actionToParent()
            else:
                return self.currentNode.getAction()

    def setNextAction(self):
        action = self.action
        if action == Dirs.NORTH:
            # if we arrived at result
            if self.y >= self.result[1] :
                self.action = self.nextAction(self.goBack)
                if action != self.action:
                    self.state = "rotate"
                self.updateResult()
        elif action == Dirs.EAST :
            if self.x >= self.result[0] :
                self.action = self.nextAction(self.goBack)
                if action != self.action:
                    self.state = "rotate"
                self.updateResult()
        elif action == Dirs.SOUTH :
            if self.y <= self.result[1] :
                self.action = self.nextAction(self.goBack)
                if action != self.action:
                    self.state = "rotate"
                self.updateResult()
        elif action == Dirs.WEST:
            if self.x <= self.result[0] :
                self.action = self.nextAction(self.goBack)
                if action != self.action:
                    self.state = "rotate"
                self.updateResult()
        elif action == Dirs.FINISH:
            return

            
    def targetRotation(self):
        if self.action == Dirs.NORTH:
            return 90.0
        elif self.action == Dirs.EAST :
            return 0.0
        elif self.action == Dirs.SOUTH :
            return -90.0
        elif self.action == Dirs.WEST :
            return 180.0
        else:
            return self.ang
    
    # The WEST angle is special since the angles are measured in [-180, 180]
    # so we need to make sure that we dont get huge variances when doing for example:
    #        : -178 - 180;
    # The simple solution is to compare to 180 when the measured angle is positive,
    # and to -180 when the measured angle is negative.
    def angVariance(self, targetRotation):
        angVariance = 0.0
        if self.action == Dirs.WEST:
            if self.ang > 0 : angVariance = self.ang - targetRotation
            else : angVariance = self.ang + targetRotation
        else :
            angVariance = self.ang - targetRotation
        return angVariance

    def rotationAdjustment(self, targetRotation):
        return self.angVariance(targetRotation) * 0.005

    def translationAdjustment(self, left, right, adjust):
        if(math.fabs(left+right) < 4):
            return 0.0
        translationVariance = left - right
        return adjust + (translationVariance*0.02)

    def updatePosition(self, lPow, rPow):
        self.outL = (lPow*0.5 + self.outL*0.5) *1
        self.outR = (rPow*0.5 + self.outR*0.5) *1
        lin = (self.outL + self.outR) / 2
        self.x = self.x + lin*math.cos(self.ang*math.pi/180)
        self.y = self.y + lin*math.sin(self.ang*math.pi/180)

    def fixPosition(self):
        measures = self.measures
        sensors = measures.irSensor
        compass = measures.compass
        try:
            distLeft = (1/sensors[LEFT])+0.5
            distRight = (1/sensors[RIGHT])+0.5
            distFront = (1/sensors[FRONT])+0.5
        except:
            return
        
        
        roundX = int(round(self.x))
        roundY = int(round(self.y))
        validCell = abs(roundX - self.x)<=0.5 and abs(roundY - self.y)<=0.5
        dir_ = selg.angVariance(self.targetRotation())
        validAngle = abs(dir_) <= 10
        if (distRight<=1.0 or distLeft<=1.0) and validCell and validAngle:
            m,M = min(distRight,distLeft), max(distRight,distLeft)
            
            if m<0.7:

                fixY = self.direction & 1
                fixX = 1 - fixY
                robotVarX = self.x - roundX
                robotVarY = self.y - roundY

                sensorVarX = (0.9-m)*[-1,1][(distRight > distLeft) == (self.action==Dirs.EAST)]
                sensorVarY = (0.9-m)*[-1,1][(distRight > distLeft) == (self.action==Dirs.SOUTH)]
                
                self.x += fixX*(sensorVarX - robotVarX)/2
                self.y += fixY*(sensorVarY - robotVarY)/2

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

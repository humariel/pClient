
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from tree_search import *
from RMI import *
from Cards import *
import math

CELLROWS=7
CELLCOLS=14

def getReversePath(actions):
    reverse = []
    for a in reversed(actions):
        if a == Cards.NORTH:
            reverse.append(Cards.SOUTH)
        elif a == Cards.EAST:
            reverse.append(Cards.WEST)
        elif a == Cards.SOUTH:
            reverse.append(Cards.NORTH)
        else:   
            reverse.append(Cards.EAST)

    return reverse

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, start, target):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.first = True
        # robot representation
        self.x = start[0]
        self.y = start[1]
        self.targetX = target[0]
        self.targetY = target[1]
        self.ang = 0
        self.outL = 0
        self.outR = 0
        # movement
        self.result = [self.x,self.y]
        self.actions = []

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
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
        self.ang = self.measures.compass

        lPow, rPow = 0.0 , 0.0
        # TODO move this to init
        if self.first:
            #search path
            d = RMI(self.labMap)
            p = SearchProblem(d, [self.x, self.y], [self.targetX, self.targetY])
            t = SearchTree(p)
            path, _, _ = t.search() 
            self.actions = path[2:] #ignore first action which is none, ignore second cause ???
            #add reverse path to go back
            self.actions += getReversePath(self.actions)
            self.actions.append(Cards.STOP)
            print(self.actions)
            self.updateResult()
            self.first = False
        
        self.setNextAction()
        print(self.actions)
        targetRotation = self.updateTargetRotation()

        if not collision:
            adjust = self.rotationAdjustment(targetRotation)
            adjust = self.translationAdjustment(left, right, adjust)
            if center > 5 :
                lPow = -0.1 + adjust
                rPow = -0.1 - adjust
            else :
                lPow = 0.1 + adjust
                rPow = 0.1 - adjust
            
            if self.actions[0] == Cards.STOP :
                lPow = 0.0
                rPow = 0.0
        else:
            lPow = -0.1
            rPow = -0.1

        self.updatePosition(lPow, rPow)
        self.driveMotors(lPow, rPow)

    def updateResult(self):
        if self.actions[0] == Cards.NORTH:
            self.result[1] += 2
        elif self.actions[0] == Cards.EAST :
            self.result[0] += 2
        elif self.actions[0] == Cards.SOUTH :
            self.result[1] -= 2
        elif self.actions[0] == Cards.WEST :
            self.result[0] -= 2

    def setNextAction(self):
        if self.actions[0] == Cards.NORTH:
            if self.y >= self.result[1] :
                self.actions.pop(0)
                self.updateResult()
        elif self.actions[0] == Cards.EAST :
            if self.x >= self.result[0] :
                self.actions.pop(0)
                self.updateResult()
        elif self.actions[0] == Cards.SOUTH :
            if self.y <= self.result[1] :
                self.actions.pop(0)
                self.updateResult()
        elif self.actions[0] == Cards.WEST:
            if self.x <= self.result[0] :
                self.actions.pop(0)
                self.updateResult()
            
    def updateTargetRotation(self):
        if self.actions[0] == Cards.NORTH:
            return 90.0
        elif self.actions[0] == Cards.EAST :
            return 0.0
        elif self.actions[0] == Cards.SOUTH :
            return -90.0
        elif self.actions[0] == Cards.WEST :
            return 180.0
        else:
            return self.ang

    
    # The WEST angle is special since the angles are measured in [-180, 180]
    # so we need to make sure that we dont get huge variances when doing for example:
    #        : -178 - 180;
    # The simple solution is to compare to 180 when the measured angle is positive,
    # and to -180 when the measured angle is negative.
    def rotationAdjustment(self, targetRotation):
        angVariance = 0.0
        if self.actions[0] == Cards.WEST:
            if self.ang > 0 : angVariance = self.ang - targetRotation
            else : angVariance = self.ang + targetRotation
        else :
            angVariance = self.ang - targetRotation
        return angVariance * 0.005

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
start = [3.0,7.0]
target = [25.0,7.0]

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--start" or sys.argv[i] == "-s") and i != len(sys.argv) - 1:
        start = [float(f) for f in sys.argv[i+1].split(',')]
    elif (sys.argv[i] == "--target" or sys.argv[i] == "-t") and i != len(sys.argv) - 1:
        target = [float(f) for f in sys.argv[i+1].split(',')]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host, start, target)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()

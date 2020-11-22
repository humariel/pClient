from tree_search import *
from Cards import Cards
import math

CELLROWS=7
CELLCOLS=14

class RMI(SearchDomain):

    def __init__(self, map=None):
        self.map = map

    def actions(self, state):
        #TODO check walls
        actions = [Cards.EAST, Cards.SOUTH, Cards.WEST, Cards.NORTH]
        if self.map:
            toRemove = []
            for a in actions:
                newPos = self.result(state, a)
                newPos = [int(x) for x in newPos]
                if newPos[0] < 0 or newPos[0] >= (CELLCOLS*2-1) or newPos[1] < 0 or newPos[1] >= CELLROWS*2-1:
                    print("{} {} is OOB".format(state, a))
                    toRemove.append(a)
                else:
                    cell = [0, 0]
                    cell[0] = (state[0]-1)//2 if state[0] > 1 else state[0]-1
                    cell[1] = (state[1]-1)//2 if state[1] > 1 else state[1]-1
                    cell = [int(x) for x in cell]
                    if cell == [6,3]:
                        print(self.map[cell[1]*2+1][cell[0]*2] != " ", a)
                        print(self.map[cell[1]*2][cell[0]*2+1] != " ", a)
                        print(self.map[cell[1]*2-1][cell[0]*2] != " ", a)
                        print(self.map[cell[1]*2][cell[0]*2-1] != " ", a)
                    if a == Cards.NORTH:
                        if self.map[cell[1]*2+1][cell[0]*2] != " ":
                            print("Wall at NORTH", cell[0], cell[1])
                            toRemove.append(a)
                    if a == Cards.EAST:
                        if self.map[cell[1]*2][cell[0]*2+1] != " ":
                            print("Wall at EAST", cell[0], cell[1])
                            toRemove.append(a)
                    if a == Cards.SOUTH:
                        if self.map[cell[1]*2-1][cell[0]*2] != " ":
                            print("Wall at SOUTH", cell[0], cell[1])
                            toRemove.append(a)
                    if a == Cards.WEST:
                        if self.map[cell[1]*2][cell[0]*2-1] != " ":
                            print("Wall at WEST", cell[0], cell[1])
                            toRemove.append(a)
            actions = [a for a in actions if a not in toRemove]

        return actions
        
    def result(self,state,action):
        if action == Cards.NORTH:
            return [state[0], state[1]+2]
        elif action == Cards.EAST:
            return [state[0]+2, state[1]]
        elif action == Cards.SOUTH:
            return [state[0], state[1]-2]
        else:
            return [state[0]-2, state[1]]

    def cost(self, state, newstate):
        return math.fabs(state[0] - newstate[0]) + math.fabs(state[1] - newstate[1])

    def heuristic(self, state, goal_state):
        return math.fabs(state[0] - goal_state[0]) + math.fabs(state[1] - goal_state[1])
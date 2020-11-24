from tree_search import *
from Dirs import Dirs
import math

CELLROWS=7
CELLCOLS=14

def opposite(action):
    if action == Dirs.NORTH:
        return Dirs.SOUTH
    elif action == Dirs.EAST:
        return Dirs.WEST
    elif action == Dirs.SOUTH:
        return Dirs.NORTH
    else:   
        return Dirs.EAST

class Challenge3(SearchDomain):

    def __init__(self, visitedNodes, walls):
        self.visitedNodes = visitedNodes
        self.walls = walls
        print(walls)

    def actions(self, state):
        #TODO check walls
        actions = [Dirs.EAST, Dirs.SOUTH, Dirs.WEST, Dirs.NORTH]
        if self.visitedNodes and self.walls:
            for a in actions:
                print(self.result(state,a))
                print(self.result(state,a) in self.visitedNodes and [self.result(state,a), opposite(a)] not in self.walls)
            actions = [a for a in actions if self.result(state,a) in self.visitedNodes and [self.result(state,a), opposite(a)] not in self.walls]
        print("-------------------")
        return actions
        
    def result(self,state,action):
        if action == Dirs.NORTH:
            return (state[0], state[1]+2)
        elif action == Dirs.EAST:
            return (state[0]+2, state[1])
        elif action == Dirs.SOUTH:
            return (state[0], state[1]-2)
        else:
            return (state[0]-2, state[1])

    def cost(self, state, newstate):
        return math.fabs(state[0] - newstate[0]) + math.fabs(state[1] - newstate[1])

    def heuristic(self, state, goal_state):
        return math.fabs(state[0] - goal_state[0]) + math.fabs(state[1] - goal_state[1])

from tree_search import *
from Cards import Cards
import math

class RMI(SearchDomain):

    def __init__(self, walls=None):
        self.walls = walls

    def actions(self):
        #TODO check walls
        return [Cards.EAST, Cards.SOUTH, Cards.WEST, Cards.NORTH]

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
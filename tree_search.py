from abc import ABC, abstractmethod

# Dominios de pesquisa
# Permitem calcular
# as accoes possiveis em cada estado, etc


class SearchDomain(ABC):

# construtor
    @abstractmethod
    def __init__(self):
        pass

# lista de accoes possiveis num estado
    @abstractmethod
    def actions(self, state):
        pass

# resultado de uma accao num estado, ou seja, o estado seguinte
    @abstractmethod
    def result(self, state, action):
        pass

# custo de uma accao num estado
    @abstractmethod
    def cost(self, state, action):
        pass

# custo estimado de chegar de um estado a outro
    @abstractmethod
    def heuristic(self, state, goal_state):
        pass

# Problemas concretos a resolver
# dentro de um determinado dominio


class SearchProblem:

    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal

    def goal_test(self, state):
        return state == self.goal

# Nos de uma arvore de pesquisa
class SearchNode:

    def __init__(self, action, state, parent, depth, cum_cost, heur):
        self.action = action
        self.state = state
        self.parent = parent
        self.depth = depth
        self.cum_cost = cum_cost
        self.heur = heur

    def __str__(self):
        return "no(" + str(self.state) + ", p=" + str(self.parent) + \
                ", d=" + str(self.depth) + ", c=" + str(self.cum_cost) + ")"

    def __repr__(self):
        return str(self)

    def inParent(self, state):
        if not self.parent:
            return False
        if self.parent.state == state:
            return True
        return self.parent.inParent(state)

# Arvores de pesquisa
class SearchTree:

# construtor
    def __init__(self,problem, strategy='a*'):
        self.problem = problem
        root = SearchNode(None,problem.initial, None, 0, 0,  0)
        self.open_nodes = [root]
        self.strategy = strategy
        self.solution_size = 0
        self.terminal_nodes = 0
        self.non_terminal_nodes = 1
        self.ratio = 0
        self.solution_cost = 0

# obter o caminho (sequencia de estados) da raiz ate um no
    def get_path(self,node):
        if node.parent == None:
            return [node.action]
        path = self.get_path(node.parent)
        path += [node.action]
        return path

# procurar a solucao
    def search(self, limit=None):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.solution_size = node.depth
                self.solution_cost = node.cum_cost
                return self.get_path(node), self.solution_size, self.solution_cost
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state, a)
                lnewnodes += [SearchNode(a,
                                        newstate,
                                         node,
                                         node.depth + 1,
                                         (node.cum_cost + self.problem.domain.cost(node.state, newstate)),
                                         self.problem.domain.heuristic(newstate, self.problem.goal))]
                if len(lnewnodes) > 0:
                    self.non_terminal_nodes += 1
                    self.terminal_nodes = len(lnewnodes) - 1
            filtered = [n for n in lnewnodes if not node.inParent(n.state) and (n.depth <= limit if limit else True)]
            self.add_to_open(filtered)
            self.update_ratio()
        return None

# juntar novos nos a lista de nos abertos de acordo com a estrategia
    def add_to_open(self,lnewnodes):
        if self.strategy == 'breadth':
            self.open_nodes.extend(lnewnodes)
        elif self.strategy == 'depth':
            self.open_nodes[:0] = lnewnodes
        elif self.strategy == 'uniform':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.cum_cost)
        elif self.strategy == 'greedy':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.heur)
        elif self.strategy == 'a*':
            self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.cum_cost + node.heur)


    def update_ratio(self):
        self.ratio = (self.terminal_nodes + self.non_terminal_nodes - 1)/self.non_terminal_nodes

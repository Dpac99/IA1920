# Andreia Pereira, n 89414 | Diogo Pacheco, n 89433 | Grupo n 15
import math
import pickle
import time
import itertools
import heapq

'''
    Class that represents a node of a graph
'''


class Node:

    def __init__(self, n, parent=None, transport=None):
        self.parent = parent  # parent node
        self.n = n  # number in map

        self.g = 0  # number of steps from init to self
        self.h = 0  # number of steps from self to goal
        self.f = 0  # sum of h and g. Heuristic value of a node

        self.transport = transport  # method of transportation used to go from parent to self

    def __eq__(self, other):
        return self.n == other.n

    def __str__(self):
        return str(self.n)

    def __repr__(self):
        return str(self.n)

    def __lt__(self, other):
        return self.f < other.f


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = []  # goal lists
        for i in goal:
            self.goal.append(Node(i, None, None))
        self.edges = model  # edges of graph
        self.paths = []  # final paths
        self.final = []  # final output
        self.agentPos = []  # current position of agents
        self.combinationList = []  # Priority queue with all possible moves
        self.prevCombinations = {}  # Dictionary to keep us from repeating configurations
        self.dist = []  # List that keeps distance from goal to all nodes. Done with bfs
        for i in goal:
            self.dist.append(self.BFS(i))
        self.limits = False  # Flag for ticket limit
        self.currentTickets = []  # Current ticket configuration

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder=False):

        self.limitexp = limitexp
        self.limitdepth = limitdepth
        self.anyorder = anyorder

        r = len(init)  # 3
        for i in range(r):
            self.paths.append([])

        # Calculates best permutation of goals and inits
        if anyorder == True:
            newGoals = list(self.switchGoals(init))
            prevGoals = list(self.goal)
            prevDist = list(self.dist)
            self.goal = []
            self.dist = []
            for index in newGoals:
                self.goal.append(prevGoals[index])
                self.dist.append(prevDist[index])

        if tickets != [math.inf, math.inf, math.inf]:
            self.limits = True

        self.aStar(init, tickets)

        l = len(self.paths[0])  # path size
        print(limitexp - self.limitexp)

        for i in range(l):
            self.final.append([[], []])
            for j in range(r):
                self.final[i][0].append(self.paths[j][i][0])
                self.final[i][1].append(self.paths[j][i][1])

        self.final[0][0] = []

        return self.final

    '''
        Function that recalculates best goals for a given current position
    '''

    def switchGoals(self, init):
        pos = []
        for i in range(len(self.goal)):
            pos.append(i)

        l = list(itertools.permutations(pos))

        min_maxD = math.inf
        min_i = 0
        for index, perm in enumerate(l):
            local_max = 0
            for init_i, goal in enumerate(perm):
                d = self.dist[goal][init[init_i]]
                if d > local_max:
                    local_max = d
            if local_max < min_maxD:
                min_maxD = local_max
                min_i = index

        return list(l[min_i])

    '''
        Implementation of A* algorithm, with heuristic value being distance from n to goal + distance from init to n
    '''

    def aStar(self, init, tickets):
        key = ""
        for i in init:
            node = Node(i, None, -1)
            self.agentPos.append(node)
            key += str(node)
        self.currentTickets = tickets
        self.prevCombinations[key] = 0

        while True:

            # Check if we reached mas depth
            maxg = 0
            for i in self.agentPos:
                if i.g > maxg:
                    maxg = i.g
            if maxg > self.limitdepth:
                break
            # end

            # If anyorder, checks if pre-calculated goals are still the best option
            if self.anyorder:
                pos = []
                for i in self.agentPos:
                    pos.append(i.n)
                newGoals = list(self.switchGoals(pos))
                if(newGoals != [0, 1, 2]):
                    prevGoals = list(self.goal)
                    prevDist = list(self.dist)
                    self.goal = []
                    self.dist = []
                    for index in newGoals:
                        self.goal.append(prevGoals[index])
                        self.dist.append(prevDist[index])
                    for move in self.combinationList:
                        maxf = 0
                        for index, node in enumerate(move[1]):
                            node.h = self.dist[index][node.n]
                            node.f = node.h + node.g
                            if node.f > maxf:
                                maxf = node.f
                        move = tuple((maxf, move[1], move[2]))

                    heapq.heapify(self.combinationList)

            # Expands all positions
            posList = []
            for index, parent in enumerate(self.agentPos):
                nodeList = []
                self.limitexp -= 1
                for edge in self.edges[parent.n]:
                    node = Node(edge[1], parent, edge[0])
                    node.g = parent.g + 1
                    node.h = self.dist[index][node.n]
                    node.f = node.h + node.g
                    nodeList.append(node)
                posList.append(nodeList)

            # Generates cartesian product of all possible moves
            preFilter = list(itertools.product(*posList))

            # Filters moves
            self.filterList(preFilter)

            # Pops move with lowest heuristic value
            move = heapq.heappop(self.combinationList)

            # Updates agents' positions and current tickets
            self.agentPos = list(move[1])
            self.currentTickets = list(move[2])

            # End is reached when one of these is true
            if len(self.combinationList) == 0 or self.checkGoals() or self.limitexp < 0:
                break

        if self.checkGoals():
            for i, p in enumerate(self.agentPos):
                current = p
                while current is not None:
                    self.paths[i].append([current.transport, current.n])
                    current = current.parent
                self.paths[i] = self.paths[i][::-1]

    '''
        A function that returns a list of possible moves, removing moves that result in collision,
        ticket overuse or moves that are already in the list but have lower heuristic value.
        Pushes all legal moves to priority queue
    '''

    def filterList(self, l):

        for move in l:
            key = ""
            move_f = 0
            new_move = []
            used_tickets = [0, 0, 0]
            move_tickets = [0, 0, 0]
            for index, node in enumerate(move):
                key += str(node)
                if node.f > move_f:
                    move_f = node.f
                used_tickets[node.transport] += 1
                if node not in (move[:index] + move[index + 1:]):
                    new_move.append(node)
            move_tickets[0] = self.currentTickets[0] - used_tickets[0]
            move_tickets[1] = self.currentTickets[1] - used_tickets[1]
            move_tickets[2] = self.currentTickets[2] - used_tickets[2]
            if len(new_move) == len(move)and (self.prevCombinations.get(key) is None or self.prevCombinations.get(key) < move_f) and (move_tickets[0] >= 0 and move_tickets[1] >= 0 and move_tickets[2] >= 0):
                heapq.heappush(self.combinationList,
                               (move_f, new_move, move_tickets))
                self.prevCombinations[key] = move_f

    '''
        Returns true if all agents' positions match those of their goals
    '''

    def checkGoals(self):
        for i, a in enumerate(self.agentPos):
            if a != self.goal[i]:
                return False
        return True

    '''
        A simple BFS that returns a list of the shortest distance (in steps) from s to all other points
    '''

    def BFS(self, s):

        visited = [-1] * (len(self.edges) + 1)

        queue = []

        queue.append(s)
        visited[s] = 0

        while queue:

            s = queue.pop(0)

            for i in self.edges[s]:
                if visited[i[1]] == -1:
                    queue.append(i[1])
                    visited[i[1]] = visited[s] + 1

        return visited

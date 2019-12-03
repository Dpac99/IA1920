import math
import pickle
import time
import itertools


class Node:

    def __init__(self, n, parent=None, transport=None, tickets=None):
        self.parent = parent
        self.n = n

        self.g = 0
        self.h = 0
        self.f = 0

        self.transport = transport

        self.tickets = list(tickets)

    def __eq__(self, other):
        return self.n == other.n


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = []
        for i in goal:
            self.goal.append(Node(i, None, None, []))
        self.edges = model
        self.vertixesPos = auxheur
        self.paths = []
        self.final = []
        self.agentPos = []
        self.combinationList = []
        self.dist = []
        for i in goal:
            self.dist.append(self.BFS(i))
        self.limits = False

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder=False):

        self.limitexp = limitexp
        self.limitdepth = limitdepth

        r = len(init)  # 3
        for i in range(r):
            self.paths.append([])

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

        for i in range(l):
            self.final.append([[], []])
            for j in range(r):
                self.final[i][0].append(self.paths[j][i][0])
                self.final[i][1].append(self.paths[j][i][1])

        return self.final

    def switchGoals(self, init):
        l = list(itertools.permutations([0, 1, 2]))

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

    def aStar(self, init, tickets):
        for i in init:
            node = Node(i, None, 0, tickets)
            self.agentPos.append(node)

        while True:

            maxg = 0
            for i in self.agentPos:
                if i.g > maxg:
                    maxg = i.g
            if maxg > self.limitdepth:
                break

            posList = []
            for index, parent in enumerate(self.agentPos):
                nodeList = []
                self.limitexp -= 1
                for edge in self.edges[parent.n]:
                    node = Node(edge[1], parent, edge[0], list(parent.tickets))
                    node.g = parent.g + 1
                    node.f = self.dist[index][node.n] + node.g
                    nodeList.append(node)
                posList.append(nodeList)

            preFilter = itertools.product(*posList)

            self.combinationList += self.filterList(preFilter)

            index = self.getLowerF(self.combinationList)

            move = self.combinationList.pop(index)
            if self.limits:
                used_tickets = [0, 0, 0]
                for i in move:
                    used_tickets[i.transport] += 1
                for i in move:
                    i.tickets[0] -= used_tickets[0]
                    i.tickets[1] -= used_tickets[1]
                    i.tickets[2] -= used_tickets[2]

            self.agentPos = list(move)
            if len(self.combinationList) == 0 or self.checkGoals() or self.limitexp < 0:
                break

        if self.checkGoals():
            for i, p in enumerate(self.agentPos):
                current = p
                while current is not None:
                    self.paths[i].append([current.transport, current.n])
                    current = current.parent
                self.paths[i] = self.paths[i][::-1]

    # Returns a new list only with moves that don't overlap agents and don't overuse tickets
    def filterList(self, l):

        result = []

        for move in l:
            new_move = []
            if self.limits:
                ticketsAux = []
                used_tickets = [0, 0, 0]
            for index, node in enumerate(move):
                if self.limits:
                    ticketsAux = node.tickets
                    used_tickets[node.transport] += 1
                if node not in (move[:index] + move[index + 1:]):
                    new_move.append(node)
            if len(new_move) == len(move) and (not self.limits or (ticketsAux[0] >= used_tickets[0] and ticketsAux[1] >= used_tickets[1] and ticketsAux[2] >= used_tickets[2])):
                if self.limits:
                    for i in new_move:
                        i.tickets = list(ticketsAux)
                result.append(new_move)

        return result

    def getLowerF(self, olist):
        final_i = 0
        minF = math.inf
        for index, move in enumerate(olist):
            move_maxF = 0
            for node in move:
                if node.f > move_maxF:
                    move_maxF = node.f
            if move_maxF < minF:
                minF = move_maxF
                final_i = index
        return final_i

    def checkGoals(self):
        for i, a in enumerate(self.agentPos):
            if a != self.goal[i]:
                return False
        return True

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

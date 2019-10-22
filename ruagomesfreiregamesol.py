import math
import pickle
import time


class Node:

    def __init__(self, n, parent=None, position=None, transport=None, tickets=None):
        self.parent = parent
        self.position = position
        self.n = n

        self.g = 0
        self.h = 0
        self.f = 0

        self.transport = transport

        self.tickets = list(tickets)

    def __eq__(self, other):
        return self.position == other.position and self.n == other.n


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):
        self.goal = []
        Node(goal[0], None, auxheur[goal[0]-1], None, [])
        self.edges = model
        self.vertixesPos = auxheur
        self.openList = []
        self.closedList = []

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):

        return self.aStar(init, tickets)

    def aStar(self, init, tickets):

        for p in init:
            self.openList.append(
                Node(p, None, self.vertixesPos[p-1], 0, tickets))

        while len(self.openList) > 0:
            index = self.getLowerF(self.openList)

            node = self.openList.pop(index)
            self.closedList.append(node)

            if node == self.goal:
                path = []
                current = node
                while current is not None:
                    path.append([[current.transport], [current.n]])
                    current = current.parent
                return path[::-1]

            preChildren = []

            l = len(self.edges[node.n])

            for i in range(l):
                child_n = self.edges[node.n][i][1]
                child_t = self.edges[node.n][i][0]
                if node.tickets[child_t] == 0:
                    continue
                child_tickets = list(node.tickets)
                child_tickets[child_t] -= 1
                new_node = Node(child_n, node,
                                self.vertixesPos[child_n-1], child_t, child_tickets)
                preChildren.append(new_node)

            for child in preChildren:
                for closed in self.closedList:
                    if child == closed:
                        continue

                child.g = node.g + 1
                child.h = (((child.position[0] - self.goal.position[0]) **
                            2) + ((child.position[1] - self.goal.position[1]) ** 2)) / ((child.transport + 1) ** 2)
                child.f = child.g + child.h
                print("Parent:", node.n, "child:", child.n, "h:",
                      child.h, "transport:", child.transport, "tickets:", child.tickets)

                for open in self.openList:
                    if child == open and child.g > open.g:
                        continue
                self.openList.append(child)

    def getLowerF(self, olist):
        current_i = 0
        l = len(olist)
        for i in range(l):
            if olist[i].f < olist[current_i].f:
                current_i = i

        return current_i

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
        for i in goal:
            self.goal.append(Node(i, None, auxheur[i-1], None, []))
        self.agentPos = []
        self.edges = model
        self.vertixesPos = auxheur
        self.openList = []
        # self.closedList = []
        self.final = []
        self.paths = []
        self.reached = []

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):

        r = len(init)
        for i in range(r):
            self.agentPos.append([])
            self.openList.append([])
            # self.closedList.append([])
            self.paths.append([])
            self.reached.append(False)

        self.aStar(init, tickets)

        l = len(self.paths[0])

        for i in range(l):
            self.final.append([[], []])
            for j in range(r):
                self.final[i][0].append(self.paths[j][i][0])
                self.final[i][1].append(self.paths[j][i][1])

        return self.final

    def aStar(self, init, tickets):

        for i, p in enumerate(init):
            node = Node(p, None, self.vertixesPos[p-1], 0, tickets)
            # self.openList[i].append(node)
            self.agentPos[i] = node

        while tickets[0] + tickets[1] + tickets[2] > 0 and not self.checkGoals():
            for agent, p in enumerate(self.agentPos):
                # print()
                node = self.agentPos[agent]
                # self.closedList[agent].append(node)

                preChildren = []

                l = len(self.edges[node.n])

                for i in range(l):
                    child_n = self.edges[node.n][i][1]
                    child_t = self.edges[node.n][i][0]
                    cond = 0 if not self.reached[agent] else 1
                    if node.tickets[child_t] == cond:
                        continue
                    child_tickets = list(node.tickets)
                    child_tickets[child_t] -= 1
                    new_node = Node(child_n, node,
                                    self.vertixesPos[child_n-1], child_t, child_tickets)
                    if new_node in self.agentPos:
                        continue
                    preChildren.append(new_node)

                for child in preChildren:
                    # for closed in self.closedList[agent]:
                    #   if child == closed:
                    #      continue

                    child.g = node.g + 1
                    child.h = (((child.position[0] - self.goal[agent].position[0]) **
                                2) + ((child.position[1] - self.goal[agent].position[1]) ** 2)) / ((child.transport + 1) ** 2)
                    child.f = child.g + child.h
                    # print("Agent:", agent, "parent:", node.n, "child:", child.n, "h:",
                    # child.h, "transport:", child.transport, "tickets:", child.tickets, "cond:", cond)

                    for o in self.openList[agent]:
                        if child == o and child.g > o.g:
                            continue
                    self.openList[agent].append(child)
                index = self.getLowerF(self.openList[agent])
                # print("Agent", agent, "moving from", node.n, "to",
                # self.openList[agent][index].n)
                self.agentPos[agent] = self.openList[agent].pop(index)

        if self.checkGoals():
            for i, p in enumerate(self.agentPos):
                current = p
                while current is not None:
                    self.paths[i].append([current.transport, current.n])
                    current = current.parent
                self.paths[i] = self.paths[i][::-1]

    def getLowerF(self, olist):
        current_i = 0
        l = len(olist)
        for i in range(l):
            if olist[i].f < olist[current_i].f:
                current_i = i

        return current_i

    def checkGoals(self):
        flag = True
        for i, p in enumerate(self.agentPos):
            # print("Comparing", [p.n, p.position], "to", [
                  # self.goal[i].n, self.goal[i].position])
            if p != self.goal[i]:
                flag = False
                self.reached[i] = False
            else:
                self.reached[i] = True
        return flag

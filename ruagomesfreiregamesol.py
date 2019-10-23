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
        self.final = []
        self.paths = []
        self.reached = []
        self.dist = []
        for i in goal:
            self.dist.append(self.BFS(i))

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):

        r = len(init)  # 3
        for i in range(r):
            self.agentPos.append([])
            self.openList.append([])
            self.paths.append([])
            self.reached.append(False)

        self.aStar(init, tickets)

        l = len(self.paths[0])  # path size

        for i in range(r):
            print(self.paths[i])

        for i in range(l):
            self.final.append([[], []])
            for j in range(r):
                self.final[i][0].append(self.paths[j][i][0])
                self.final[i][1].append(self.paths[j][i][1])

        return self.final

    def aStar(self, init, tickets):

        for i, p in enumerate(init):
            node = Node(p, None, self.vertixesPos[p-1], 0, tickets)
            self.agentPos[i] = node
            self.openList[i].append(node)

        while tickets[0] + tickets[1] + tickets[2] > 0 and not self.checkGoals():
            print()
            for agent, p in enumerate(self.agentPos):

                index = self.getLowerF(
                    self.openList[agent], self.reached[agent])
                # print("Agent", agent, "moving from", node.n, "to",
                #      self.openList[agent][index].n)

                self.agentPos[agent] = self.openList[agent].pop(index)

                if p.parent is not None:
                    print("Agent", agent, "in position",
                          p.n, "from", p.parent.n)

                node = self.agentPos[agent]

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

                    child.g = node.g + 1
                    child.f = self.dist[agent][child.n] + child.g
                #    print("Agent:", agent, "parent:", child.parent.n, "child:", child.n, "cost:", child.f,
                #          "transport:", child.transport, "tickets:", child.tickets, "cond:", cond)

                    for o in self.openList[agent]:
                        if child == o and child.g > o.g:
                            continue
                    self.openList[agent].append(child)

        if self.checkGoals():
            for i, p in enumerate(self.agentPos):
                current = p
                while current is not None:
                    self.paths[i].append([current.transport, current.n])
                    current = current.parent
                self.paths[i] = self.paths[i][::-1]

    def getLowerF(self, olist, flag):
        current_i = -1
        l = len(olist)
        for i, p in enumerate(olist):
            if p not in self.agentPos:
                current_i = i
                break
        for i in range(l):
            if flag:
                if olist[i].f == 1 and olist[i] not in self.agentPos:
                    return i

            if olist[i].f < olist[current_i].f and olist[i] not in self.agentPos:
                current_i = i

        return current_i

    def checkGoals(self):
        flag = True
        for i, p in enumerate(self.agentPos):
            if p != self.goal[i]:
                flag = False
                self.reached[i] = False
            else:
                self.reached[i] = True
        return flag

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

# Andreia Pereira, 89414
# Diogo Pacheco, 89433

import random
import math

# LearningAgent to implement
# no knowledeg about the environment can be used
# the code should work even with another environment


class LearningAgent:

    # init
    # nS maximum number of states
    # nA maximum number of action per state
    def __init__(self, nS, nA):

        self.nS = nS
        self.nA = nA
        self.Q = [[0 for i in range(self.nA)]
                  for j in range(self.nS + 1)]  # [nS][nA]
        # Possible actions for every state
        self.actions = [[0] for i in range(nS + 1)]
        self.count = [[-1 for i in range(self.nA)]
                      for j in range(self.nS + 1)]  # Number of times a pair (st,a) has been chosen
        self.alpha = 0.9    # learning rate
        self.gamma = 0.4   # discount factor
        self.epsilon = 1  # probability of randomness when choosing action

    # Select one action, used when learning
    # st - is the current state
    # aa - is the set of possible actions
    # for a given state they are always given in the same order
    # returns
    # a - the index to the action in aa
    def selectactiontolearn(self, st, aa):
        # print("select one action to learn better")
        a = 0
        b = random.random()

        if(self.actions[st][0] == 0):
            self.actions[st] = aa.copy()
            for i in range(len(aa)):
                self.count[st][i] = 0

        if (b >= self.epsilon):
            a = self.maxQ(st, aa)
        else:
            #a = random.randrange(0, len(aa), 1)
            a = self.minCount(st, aa)

        if (self.epsilon > 0.1):
            self.epsilon -= 0.01

        self.count[st][a] += 1

        return a

    # Select one action, used when evaluating
    # st - is the current state
    # aa - is the set of possible actions
    # for a given state they are always given in the same order
    # returns
    # a - the index to the action in aa
    def selectactiontoexecute(self, st, aa):

        return self.maxQ(st, aa)

    # this function is called after every action
    # st - original state
    # nst - next state
    # a - the index to the action taken
    # r - reward obtained
    def learn(self, ost, nst, a, r):

        alfa = self.alpha/math.log(self.count[ost][a] + math.e - 1)

        next_max = self.Q[nst][self.maxQ(nst, self.actions[nst])]

        self.Q[ost][a] = self.Q[ost][a] + alfa * \
            (r + self.gamma*next_max - self.Q[ost][a])
        return

    # returns the index of the action with max Q value
    def maxQ(self, st, aa):
        max_action = self.Q[st][0]
        max_i = 0
        n_actions = len(aa)
        for i in range(n_actions):
            if (self.Q[st][i] > max_action):
                max_action = self.Q[st][i]
                max_i = i
        return max_i

    # returns the index of the action that has been chosen
    # the least amount of times
    def minCount(self, st, aa):
        min_count = self.count[st][0]
        min_i = 0
        n_actions = len(aa)
        for i in range(n_actions):
            if (self.count[st][i] >= 0 and self.count[st][i] < min_count):
                min_count = self.count[st][i]
                min_i = i
        return min_i

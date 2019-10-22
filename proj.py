import math


class SearchProblem:

    # goal: list with goal position
    # model: graph representing position transitions
    # auxheur: array with positions coordinates
    def __init__(self, goal, model, auxheur=[]):
        pass

    # init: list with inital positons
    # limitexp: limit of expansions
    # limitdepth: Depth limit
    # tickets: list with number of avaliable tickets
    def search(self, init, limitexp=2000, limitdepth=20, tickets=[math.inf, math.inf, math.inf]):
        pass

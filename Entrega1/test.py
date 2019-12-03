import itertools
import math

dist = [[0, 0, 0, 9, 14, 19], [0, 0, 0, 7, 12, 17], [0, 0, 0, 2, 7, 12]]

init = [5, 4, 3]
goals = [1, 3, 8]
print(dist[0][5], dist[1][4], dist[2][3])

l = list(itertools.permutations([0, 1, 2]))

res = []

min_maxD = math.inf
min_i = 0
for index, perm in enumerate(l):
    local_max = 0
    for init_i, goal in enumerate(perm):
        d = dist[goal][init[init_i]]
        if d > local_max:
            local_max = d
    if local_max < min_maxD:
        min_maxD = local_max
        min_i = index

return list(l[min_i])

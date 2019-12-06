#!/usr/bin/env python3
import NDtester
import random

grade = NDtester.test()
i = 1
b = 0

while grade == 20:
    grade = NDtester.test()
    i += 1
    if i % 100 == 0:
        print("Yay! You've completed " + str(i) + " tests with grade 20!")


print("Too bad! Your agent completed " + str(i) + " tests.")

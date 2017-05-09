""" Filter out trials from argv[1] that have 'traveled' more than threshold """

import sys

bestDistances = []
threshold = 80.0 # Score (distance traveled)

try:
    f = open(sys.argv[1], 'r')
    for line in f: 
        if (float(line.partition(',')[0]) < threshold):
            bestDistances.append(line)
finally:
    f.close()

for p in bestDistances: sys.stdout.write(p)


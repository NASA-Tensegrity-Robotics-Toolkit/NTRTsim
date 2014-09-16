""" Filter out the best trials from argv[1] as dictated by their distance traveled """

import sys

bestDistances = []
threshold = 20.0 # Score (distance traveled)

try:
    f = open(sys.argv[1], 'r')
    for line in f: 
        if (float(line.partition(',')[0]) > threshold):
            bestDistances.append(line)
finally:
    f.close()

for p in bestDistances: sys.stdout.write(p)


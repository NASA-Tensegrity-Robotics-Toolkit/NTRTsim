""" Filter out the best episodes from argv[1] as dictated by their distance traveled """
""" Note: prints entire episode data, including parameters, as opposed to just distances """

import sys

bestDistances = []
threshold = 25.0 # Score (distance traveled)

try:
    f = open(sys.argv[1], 'r')
    for line in f: 
        if (float(line.partition(',')[0]) > threshold):
            bestDistances.append(line)
finally:
    f.close()

for p in bestDistances: sys.stdout.write(p)


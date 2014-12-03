""" Sort the episodes by their score (column 1) """
""" Note: prints entire episode data, including parameters, as opposed to just distances """

import sys
import csv
from operator import itemgetter

sortedDistances = []
unsorted = []
threshold = 25.0 # Score (distance traveled)

try:
    f = open(sys.argv[1], 'r')
    reader = csv.reader(f, delimiter=',')
    for row in reader:
        score = row[0]
        row[0] = float(score)
        unsorted.append(row)
finally:
    f.close()

sortedDistances = sorted(unsorted, key=lambda elem: elem[0], reverse=True)

for p in sortedDistances: 
    p[0] = '%.5f' % p[0]
    sys.stdout.write(','.join(p))
    sys.stdout.write('\n')


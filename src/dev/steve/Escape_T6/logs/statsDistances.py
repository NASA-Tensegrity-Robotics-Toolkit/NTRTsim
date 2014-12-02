""" Prints the min, avg, max of the numbers on each line """

import sys

try:
    f = open(sys.argv[1], 'r')
    lines = 0
    sum = 0
    max = 0
    min = 9999999
    for line in f:
        lines += 1
        x = float(line.partition(',')[0])
        sum += x
        if x < min:
            min = x
        if x > max:
            max = x
    print "min: " + str(min)
    print "avg: " + str((sum / lines))
    print "max: " + str(max)
finally:
    f.close()


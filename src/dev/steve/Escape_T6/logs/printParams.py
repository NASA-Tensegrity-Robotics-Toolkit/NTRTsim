""" Prints all but the first two comma-separated values on every line """
""" Input must be comma separated, output will be comma separated """

import sys

try:
    f = open(sys.argv[1], 'r')
    for line in f:
        for i in range(2, len(line.split(','))):
            if (i < len(line.split(','))-1):
                sys.stdout.write(line.split(',')[i] + ",")
            else:
                sys.stdout.write(line.split(',')[i])
finally:
    f.close()


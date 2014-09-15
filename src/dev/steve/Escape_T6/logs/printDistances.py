""" Prints the first comma-separated value on every line """

import sys 

f = open(sys.argv[1], 'r') 

try:
    for line in f:
        print line.partition(',')[0] # Implicit '\n' printed as well
finally:
    f.close()
 

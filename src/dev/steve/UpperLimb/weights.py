# uniformly distributed

import random         
import sys

nWeights = 250

for i in range (0,nWeights):
    sys.stdout.write("%f" % ((random.random()*2)-1))
    if (i < nWeights-1):
        print ",",


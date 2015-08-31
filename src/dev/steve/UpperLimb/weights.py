# uniformly distributed

import random         
import sys

nInputs = 3
nHidden = 10
nOutputs = 22
nWeights = (nInputs+1)*nHidden + (nHidden+1)*nOutputs # follow nnv2 format

for i in range (0,nWeights):
    sys.stdout.write("%f" % ((random.random()*2)-1))
    if (i < nWeights-1):
        print ",",


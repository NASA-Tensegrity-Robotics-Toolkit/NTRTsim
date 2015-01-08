#!/usr/bin/python
""" Prints statistics about a run """

import sys
import csv
from operator import itemgetter
import numpy as np

def statScores(inFile):
    dists = []
    try:
        f = open(inFile, 'r')
        for line in f: 
            dists.append(float(line.partition(',')[0]))
    finally:
        f.close()

    run_mean = np.mean(dists)
    run_std = np.std(dists)
    run_var = np.var(dists)

    print "Mean: ",run_mean,", STD: ",run_std,", VAR: ",run_var

if __name__=="__main__":
    inFile = sys.argv[1]
    statScores(inFile)


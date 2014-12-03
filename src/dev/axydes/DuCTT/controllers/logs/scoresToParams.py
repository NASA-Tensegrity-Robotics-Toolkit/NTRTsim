#!/usr/bin/python
""" Goes through entire workflow to get MonteCarlo params for learning algorithm """

import sys
import csv
from operator import itemgetter

def sortFile(inFile, outFile):
    sortedDistances = []
    unsorted = []
    threshold = 25.0 # Score (distance traveled)

    try:
        f = open(inFile, 'r')
        reader = csv.reader(f, delimiter=',')
        for row in reader:
            score = row[0]
            row[0] = float(score)
            unsorted.append(row)
    finally:

        f.close()

    sortedDistances = sorted(unsorted, key=lambda elem: elem[0], reverse=True)

    try:
        f = open(outFile, 'w')
        writer = csv.writer(f, delimiter=',')
        for p in sortedDistances: 
            p[0] = '%.5f' % p[0]
            writer.writerow(p)
    finally:
        f.close()

    return

def cutOuts(inFile, outFile, threshold):
    # threshold = Score (distance traveled)
    bestDistances = []

    try:
        f = open(inFile, 'r')
        for line in f: 
            if (float(line.partition(',')[0]) < threshold):
                bestDistances.append(line)
    finally:
        f.close()

    try:
        f = open(outFile, 'w')
        for p in bestDistances: 
            f.write(p)
    finally:
        f.close()

    return

def bestScores(inFile, outFile, threshold):
    # threshold = Score (distance traveled)
    bestDistances = []

    try:
        f = open(inFile, 'r')
        for line in f: 
            if (float(line.partition(',')[0]) > threshold):
                bestDistances.append(line)
    finally:
        f.close()

    try:
        f = open(outFile, 'w')
        for p in bestDistances: 
            f.write(p)
    finally:
        f.close()

    return

def printParams(inFile, outFile):
    try:
        fin = open(inFile, 'r')
        fout = open(outFile, 'w')
        for line in fin:
            for i in range(2, len(line.split(','))):
                if (i < len(line.split(','))-1):
                    fout.write(line.split(',')[i] + ",")
                else:
                    fout.write(line.split(',')[i])
    finally:
        fin.close()
        fout.close()

    return

if __name__=="__main__":
    scoreFile = sys.argv[1]
    sortedFile = 'sorted_'+scoreFile
    noOutsFile = 'sorted_NoOutliers_'+scoreFile
    bestFile = 'best_'+scoreFile
    bestParamFile = 'bestParams_'+scoreFile

    sortFile(scoreFile, sortedFile)
    cutOuts(sortedFile, noOutsFile, 200)
    bestScores(noOutsFile, bestFile, 80)
    printParams(bestFile, bestParamFile)


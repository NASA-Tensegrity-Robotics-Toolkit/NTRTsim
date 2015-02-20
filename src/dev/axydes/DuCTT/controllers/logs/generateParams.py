#!/usr/bin/python
""" Goes through entire workflow to get MonteCarlo params for learning algorithm """

import os
import sys
import csv
import re
import shutil
from operator import itemgetter
import numpy as np
import statScores

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

    sortedEnergy = sorted(unsorted, key=itemgetter(1))
    sortedDistances = sorted(sortedEnergy, key=itemgetter(0), reverse=True)

    try:
        f = open(outFile, 'w')
        writer = csv.writer(f, delimiter=',')
        for p in sortedDistances: 
            p[0] = '%.5f' % p[0]
            writer.writerow(p)
    finally:
        f.close()

    return

def cutOuts(inFile, outFile):
    # threshold = Score (distance traveled)
    bestDistances = []

    dists = []
    try:
        f = open(inFile, 'r')
        for line in f: 
            dists.append(float(line.partition(',')[0]))
    finally:
        f.close()

    mean = np.mean(dists)
    std = np.std(dists)

    lowerThresh = mean - 2*std
    upperThresh = mean + 2*std

    if mean <= 0:
        upperThresh = np.max(dists)
        lowerThresh = 0

    try:
        f = open(inFile, 'r')
        for line in f: 
            dist = float(line.partition(',')[0])
            if (dist <= upperThresh and dist >= lowerThresh):
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

def createFolder(trialFolder):
    if (not os.path.exists(trialFolder)):
        os.makedirs(trialFolder)

    if not trialFolder.endswith('/'):
        trialFolder = trialFolder+'/'

    regex = re.compile('trial_([0-9])+$')

    maxTrial = 0
    currTrials = os.listdir(trialFolder)
    for trial in currTrials:
        matched = regex.match(trial)
        if matched:
            newTrial = int(matched.group(1))
            if newTrial >= maxTrial:
                maxTrial = newTrial+1

    newTrialDir = trialFolder+'trial_'+str(maxTrial)
    print 'Trial folder: {}'.format(newTrialDir)
    os.makedirs(newTrialDir)

    return newTrialDir

def mainFunc(inFile, trialFolder):
    if not os.path.exists(inFile):
        print 'Error: {} does not exist.'.format(inFile)
        return

    #Setup trial forlder
    folder = createFolder(trialFolder)

    # Create filenames
    scoreFile = folder+'/'+inFile 
    sortedFile = folder+'/'+'sorted_'+inFile
    noOutsFile = folder+'/'+'sorted_NoOutliers_'+inFile
    bestFile = folder+'/'+'best_'+inFile
    bestParamFile = folder+'/'+'bestParams_'+inFile

    # Move file to new trial folder
    shutil.move(inFile,scoreFile)
    shutil.copy2('../Config.ini',folder+'/Config.ini')

    #Do the actual parameter generation
    sortFile(scoreFile, sortedFile)
    # shutil.copy2(sortedFile,noOutsFile)
    cutOuts(sortedFile, noOutsFile)
    bestScores(noOutsFile, bestFile, 0)
    printParams(bestFile, bestParamFile)
    statScores.statScores(bestFile)

    return

if __name__=="__main__":
    if len(sys.argv) < 2:
        print 'Usage: {} [SCORES_FILE] [TRIAL_FOLDER]'.format(os.path.basename(sys.argv[0]))
        exit(-1)
    if len(sys.argv) < 3:
        folder = '.'
    else:
        folder = sys.argv[2]

    mainFunc(sys.argv[1], folder)


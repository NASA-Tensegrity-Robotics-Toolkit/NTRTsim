#!/usr/bin/python

# Copyright (c) 2012, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All rights reserved.
#
# The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
# under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0.
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

""" Goes through entire workflow to get MonteCarlo params for learning algorithm """

# Purpose: Assist users with learning trials and playback of learned parameters
# Author:  Brian Mirletz, Alexander Xydes
# Date:    January 2015
# Notes:   Converts a scores.csv file into the '.nnw' files uses by
# the learning library. Input parameters are
# (1) The name of the scores file, must be "single origin" See
#     splitInfile.py for preliminary work that may need to be done
# (2) The name of the Config.ini file, may be in a different directory
# (3) The best number of scores files to output
# (4) Any suffix that needs to be applied to bestParams files

import sys
import csv
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

def printParams(inFile, suffix, numActions, numControllers, numScores):
    try:
        fin = open(inFile, 'r')
        for i in range(0, numScores):
            line = fin.readline()
            for j in range(0, numControllers):
                # Format requried by learning libraries
                if suffix != None:
                    outFile = 'bestParameters-' + str(i) + '_' + suffix + '-' + str(j) + '.nnw'
                else:
                    outFile = 'bestParameters-' + str(i) + '-' + str(j) + '.nnw'
                fout = open(outFile, 'w')
                for k in range(0, numActions):
                    # Iterate down the line of elements. +2 covers the score at the beginning
                    index = numActions * j + k + 2;
                    if (k < numActions-1):
                        fout.write(line.split(',')[index] + ",")
                    else:
                        fout.write(line.split(',')[index])

    finally:
        fin.close()
        fout.close()

    return

def getActionValues(configFile):
    try:
        f = open(configFile, 'r')
        for line in f:
            if line.partition('=')[0] == 'numberOfActions':
                act = int(line.partition('=')[2])
            if line.partition('=')[0] == 'numberOfControllers':
                cont = int(line.partition('=')[2])
    finally:
        f.close()
    return act, cont


if __name__=="__main__":
    scoreFile = sys.argv[1]
    configFile = sys.argv[2]
    numScores = int(sys.argv[3])
    if len(sys.argv) == 5:
        suffix = sys.argv[4]
    else:
        suffix = None

    sortedFile = 'sorted_'+scoreFile
    topFile = 'sorted_top_'+scoreFile

    nums = getActionValues(configFile)
    sortFile(scoreFile, sortedFile)
    printParams(sortedFile, suffix, nums[0], nums[1], numScores)


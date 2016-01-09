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

""" Converts .nnw files to a JSON file """

# Purpose: Read JSON files and print results to the command line. Used to determine best params after learning has completed
# Author:  Brian Mirletz
# Date:    March 2015

import sys
import os
import subprocess
import json
import random
import logging
import collections
import operator


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    configFile = sys.argv[1]
    numFiles = int(sys.argv[2])
    # The particular set of parameters you want to sort by. Used mainly as a unique identifier for sorting the controllers.
    #should be the name of an existing parameter set from the JSON spec file, and it is assumed that 'Vals' is the suffix of this name. 
    paramType = sys.argv[3] 
    if len(sys.argv) == 5:
        numScore = int(sys.argv[4])
    else:
        numScore = 1

    scoreSum = 0
    fileSum = 0
    maxScore = -1000
    topScore = []
    topParam = []

    paramList = []

    paramVals = paramType + 'Vals'

    for i in range(0, numFiles):
        scoresPath = configFile + str(i) +'.json'
        fin = open(scoresPath, 'r')
        obj = json.load(fin)
        fin.close()
        try:
            paramID = obj[paramVals]['paramID']  

            if (paramList.count(paramID) == 0):
                # Use this for processing monteCarlo
                 
                thisScore = 0
                for k in range(0, 4):
                    try:
                        thisScore += float(obj['scores'][k]['distance'])
                    except IndexError:
                        thisScore += 0
                """
                
                thisScore = float(obj[paramVals]['avgScore'])
                """
                fileSum += 1
                paramList.append(paramID)
            """
            else:
                print(str(i) + " " +str(paramID) + " " + str(paramList.count(paramID)))
                thisScore = 0
            """
        except KeyError:
            print(str(i) + " " + str(KeyError))
            thisScore =  0

        if(thisScore > maxScore):
            j = min([numScore, len(topScore)])
            # Sort into position
            while j > 0:
                if thisScore > topScore[j - 1]:
                    j -= 1
                else:
                    break

            topScore.insert(j, thisScore)
            topParam.insert(j, i)

            if (len(topScore) > numScore):
                maxScore = topScore[numScore - 1]
                topScore.pop(numScore)
                topParam.pop(numScore)
            topObj = obj

        scoreSum += thisScore

    print(maxScore)
    print(scoreSum / fileSum)
    print(topScore)
    print(topParam)

    # Now average the scores of the top object


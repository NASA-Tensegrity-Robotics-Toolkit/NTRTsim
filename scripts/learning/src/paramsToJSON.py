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

# Purpose: Assist users with learning trials and playback of learned parameters
# Author:  Brian Mirletz
# Date:    March 2015
# Notes:   Converts .nnw files to .json files as specified by users. Inputs include
# (1) The name of the config.ini file
# (2) The path to the .nnw files
# (3) The full name of the desired JSON output file
# (4) The name of the controller (default, 1, etc). 'suffix' in C++ parliance
# (5) The variable name that will be used in JSON
# (6) (optional) any additional suffix, assumed to be after an _ in the file name (such as node or edge)
# (7) (optional) an input JSON file, if the name is different than the output file

import sys
import csv
from operator import itemgetter
import numpy as np
import json

def printParams(path, controllerNum, suffix, numActions, numControllers):

    dataArray = []

    for i in range(0, numControllers):
        # Format requried by learning libraries
        if suffix != None:
            inFile = path + 'bestParameters-' + controllerNum + '_' + suffix + '-' + str(i) + '.nnw'
        else:
            inFile = path + 'bestParameters-' + controllerNum + '-' + str(i) + '.nnw'

        # We want to fail if this file doesn't exist
        fin = open(inFile, 'r')
        subList = []
        line = fin.readline()
        for k in range(0, numActions):
            subList.append(float(line.split(',')[k]))
        fin.close()

        print(subList)
        dataArray.append(subList)

    return dataArray

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
    configFile = sys.argv[1]
    path = sys.argv[2]
    outFile = sys.argv[3]
    controllerNum = sys.argv[4]
    varName = sys.argv[5]
    if len(sys.argv) >= 7:
        suffix = sys.argv[6]
        if len(sys.argv) == 8:
            inFile = sys.argv[7]
        else:
            inFile = outFile
    else:
        suffix = None
        inFile = outFile

    # Load existing JSON dictionary
    try:
        fin = open(inFile, 'r')
        try:
            obj = json.load(fin)
        except ValueError:
            obj = {}
        fin.close()
    except IOError:
        obj = {}



    nums = getActionValues(configFile)
    dataArray = printParams(path, controllerNum, suffix, nums[0], nums[1])

    print(dataArray)

    obj[varName] = dataArray

    fout = open(outFile, 'w')

    json.dump(obj, fout, indent=4)

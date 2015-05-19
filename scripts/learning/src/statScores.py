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

""" Prints statistics about a run """

# Purpose: Print statistics about a learning trial.
# Author:  Alexander Xydes
# Date:    January 2015
# Notes:   Prints statistics about both scores from a scores.csv file.
# Input parameters are
# (1) The name of the scores file, must be "single origin" See
#     splitInfile.py for preliminary work that may need to be done

import sys
import csv
import numpy as np

def statScores(inFile):
    scores1 = []
    scores2 = []
    try:
        f = open(inFile, 'r')
        reader = csv.reader(f, delimiter=',')
        for row in reader:
            scores1.append(float(row[0]))
            scores2.append(float(row[1]))
    finally:
        f.close()

    mean1 = np.mean(scores1)
    std1 = np.std(scores1)
    var1 = np.var(scores1)
    mean2 = np.mean(scores2)
    std2 = np.std(scores2)
    var2 = np.var(scores2)

    print "Score 1:\nMean: {:.6g}, STD: {:.6g}, VAR: {:.6g}\n".format(mean1, std1, var1)
    print "Score 2:\nMean: {:.6g}, STD: {:.6g}, VAR: {:.6g}\n".format(mean2, std2, var2)

if __name__=="__main__":
    inFile = sys.argv[1]
    statScores(inFile)


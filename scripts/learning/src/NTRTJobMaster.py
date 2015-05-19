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

# Purpose: Queue up learning runs, pass them parameters via JSON. Can run several different types of learning.
# Author:  Brian Mirletz and Perry Bhandal
# Date:    March 2015
# Notes:   In progress as of this commit (3/27/15)

import sys
import os
import subprocess
import json
import random
import logging
import collections
import operator
from interfaces import NTRTJobMaster, NTRTJob, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler

###
# Interfaces.
##


###
# Brian's implementations.
# @todo consider moving this to a different file
###

class LastUpdatedOrderedDict(collections.OrderedDict):
    'Store items in the order the keys were last added'

    def __setitem__(self, key, value):
        if key in self:
            del self[key]
        collections.OrderedDict.__setitem__(self, key, value)

class BrianJobMaster(NTRTJobMaster):
    def _setup(self):
        """
        Read input file, store file paths for run
        """

        # If this fails, the program should fail. Input file is required
        # for useful output
        try:
            fin = open(self.configFileName, 'r')
            self.jConf = json.load(fin)
            fin.close()
        except IOError:
            raise NTRTMasterError("Please provide a valid configuration file")

        self.path = self.jConf['resourcePath'] + self.jConf['lowerPath']

        try:
            os.makedirs(self.path)
        except OSError:
            if not os.path.isdir(self.path):
                raise NTRTMasterError("Directed the folder path to an invalid address")

        # Consider seeding random, using default (system time) now
        #random.seed(5)

    def __writeToNNW(self, neuralParams, fileName):
        """
        Take the params (neuralParams) and write them to a .nnw file for reading by
        the neuralNetwork code (Third party library)
        """
        fout = open(fileName, 'w')
        first = True
        for x in neuralParams:
            if (first):
                fout.write(str(x))
                first = False
            else:
                fout.write("," + str(x))


    def __getNewParams(self, paramName):
        """
        Generate a new set of paramters based on learning method and config file
        Returns a dictionary paramName : values
        This is the implementation of monteCarlo
        """
        params = self.jConf["learningParams"][paramName]

        pMax = params['paramMax']
        pMin = params['paramMin']

        newController = {}
        newController['paramID'] = str(self.paramID)

        # Neural network or not. Slightly different data structure
        if params['numberOfStates'] == 0 :

            newParams = []

            for i in range(0, params['numberOfInstances']) :
                subParams = []
                for j in range(0, params['numberOfOutputs']) :
                    # Assume scaling happens elsewhere
                    subParams.append(random.uniform(pMin, pMax))
                newParams.append(subParams)
        else :
            newParams = { 'numActions' : params['numberOfOutputs'],
                         'numStates' : params['numberOfStates'],
                         'numHidden' : params['numberHidden'],
                         'neuralFilename' : "logs/bestParameters-test_fb-"+ newController['paramID'] +".nnw"}

            neuralParams = []

            numStates = params['numberOfStates']
            numOutputs=  params['numberOfOutputs']
            numHidden = params['numberHidden']

            totalParams = (numStates + 1) * (numHidden) + (numHidden + 1) * numOutputs

            for i in range(0,  totalParams) :
                neuralParams.append(random.uniform(pMin, pMax))

            newParams['neuralParams'] = neuralParams

            self.__writeToNNW(neuralParams, self.path + newParams['neuralFilename'])

        newController['params'] = newParams
        newController['scores'] = []

        return newController

    def __mutateParams(self, currentController, paramName):
        """
        Change some parameters (according to mutationChance) by an amount sampled from a gaussian
        (mutationDev)
        """
        params = self.jConf["learningParams"][paramName]

        pMax = params['paramMax']
        pMin = params['paramMin']

        # Slightly different encoding for neural nets
        if(params['numberOfStates'] > 0):
            cNew = list(currentController['neuralParams'])
        else:
            # Make a deep copy
            cNew = list(currentController)

        for i, v in enumerate(cNew):
            if isinstance(v, collections.Iterable):
                v = self.__mutateParams(v, paramName)
            else:
                if(random.random() > params['mutationChance']):

                    mutAmount = random.normalvariate(0.0, params['mutationDev'])
                    v += mutAmount

                    if (v > pMax):
                        v = pMax
                    elif (v < pMin):
                        v = pMin
            cNew[i] = v

        # Slightly different encoding for neural nets
        if(params['numberOfStates'] > 0):
            newNeuro = {}
            newNeuro['neuralParams'] = cNew
            newNeuro['numStates'] = params['numberOfStates']
            newNeuro['numActions'] =  params['numberOfOutputs']
            newNeuro['numHidden'] =  params['numberHidden']
            return newNeuro
        else:
            return cNew

    def __getControllerFromProbability(self, currentGeneration, prob):
        """
        A support function for genetic algorithms that selects a controller
        based on the distribution of probabilities for all of their controllers (their
        contriution to the total score of the generation)
        """
        for c in currentGeneration.itervalues():
            if (c['probability'] < prob):
                break

        return c

        raise NTRTMasterError("Insufficient values to satisfy requested probability")

    def __getChildController(self, c1, c2, params):
        """
        Takes two controllers and merges them with a 50/50 chance of selecting
        a parameter from each controller
        """

        if(params['numberOfStates'] > 0):
            c1 = list(c1['neuralParams'])
            c2 = list(c2['neuralParams'])

        cNew = []

        if (len(c1) == 0):
            raise NTRTMasterError("Error in length")

        """
        # Old code for random params
        for i, j in zip(c1, c2):
            # Go to the deepest level of the parameters (see instances in the specification)
            if isinstance(i, collections.Iterable):
                cNew.append(self.__getChildController(i, j, params))
            else:
                # @todo should this be adjustable?
                if (random.random() > 0.5):
                    cNew.append(i)
                else:
                    cNew.append(j)
        """
        crossOver = random.randint(0, len(c1) - 1)

        print(crossOver)

        cNew[0:crossOver] = c1[0:crossOver]
        print(len(cNew))

        cNew[crossOver:len(c2)+1] = c2[crossOver:len(c2)+1]
        print(len(cNew))


        if (len(cNew) != len(c1)):
            raise NTRTMasterError("Error in length")

        if(params['numberOfStates'] > 0):
            newNeuro = {}
            newNeuro['neuralParams'] = cNew
            newNeuro['numStates'] = params['numberOfStates']
            newNeuro['numActions'] =  params['numberOfOutputs']
            newNeuro['numHidden'] =  params['numberHidden']
            return newNeuro
        else:
            return cNew

    def __sortAvg(self, x):
        """
        Returns the average score from a controller for sorting
        """

        return x['avgScore']

    def __sortMax(self, x):
        """
        Returns the max score of a controller for sorting
        """

        return x['maxScore']

    def generationGenerator(self, currentGeneration, paramName):
        """
        Master function that takes an existing set of paramters, sorts them by score
        and then returns a new set of parameters based on the specification file
        """

        params = self.jConf["learningParams"][paramName]

        useAvg = params['useAverage']

        nextGeneration = LastUpdatedOrderedDict()

        # Are we doing monteCarlo or starting a new trial?
        if (len(currentGeneration) == 0 or params['monteCarlo']):

            # Starting a new trial - load previous results, if any, unless doing monteCarlo
            if (not (params['learning'] and params['monteCarlo'])):
                for i in range(0, params['startingControllers']):
                    inFile = self.path + self.jConf['filePrefix'] + "_" + str(i) + self.jConf['fileSuffix']
                    # We want the IO error if this fails
                    fin = open(inFile, 'r')
                    jControl = json.load(fin)
                    fin.close()
                    controller = {}
                    # Check how controller was generated
                    try:
                        controller['params'] = jControl[paramName]['params']
                    except KeyError:
                        controller['params'] = jControl[paramName]
                    except TypeError:
                        controller['params'] = jControl[paramName]
                    controller['paramID'] = str(self.paramID)
                    controller['scores'] = []
                    nextGeneration.__setitem__(controller['paramID'], controller)

                    self.paramID += 1

            # Fill in remaining population with random parameters
            for i in range(len(nextGeneration), params['populationSize']):
                if (i < 0):
                    raise NTRTMasterError("Number of controllers greater than population size!")

                controller = self.__getNewParams(paramName)
                nextGeneration.__setitem__(controller['paramID'], controller)
                self.paramID += 1

        elif (not params['learning']):
            # Not learning, return previous controllers
            nextGeneration = currentGeneration

        else:
            # learning, not doing monteCarlo, have a previous generation

            popSize = len(currentGeneration)

            # order the prior population

            for k, controller in currentGeneration.iteritems() :
                scores = controller['scores']
                controller['maxScore'] = max(scores)
                controller['avgScore'] = sum(scores) / float(len(scores))


            if (useAvg):
                key = lambda x: x[1]['avgScore']
            else:
                key = lambda x: x[1]['maxScore']
            sortedGeneration = collections.OrderedDict(sorted(currentGeneration.items(), None, key, True))

            scoreDump = open('scoreDump.txt', 'a')
            scoreDump.write(json.dumps(sortedGeneration))
            scoreDump.write('\n')
            scoreDump.close()

            totalScore = 0

            # Get probabilities for children (for mating)

            if (useAvg):

                for controller in sortedGeneration.itervalues():
                    pass

                floor = controller['avgScore']

                for controller in sortedGeneration.itervalues():
                    totalScore += controller['avgScore'] - floor

                first = True
                c1 = {}

                # All scores are the same for some reason, don't divide by zero
                if totalScore == 0.0:
                    totalScore = 1

                for c in sortedGeneration.itervalues():
                    if first:
                        c['probability'] = (c['avgScore'] - floor) / totalScore
                        first = False
                    else:
                        c['probability'] = (c['avgScore'] - floor) / totalScore + c1['probability']
                    c1 = c

            else:
                for controller in sortedGeneration.itervalues():
                    pass

                floor = controller['maxScore']

                for controller in sortedGeneration.itervalues():
                    totalScore += controller['maxScore'] - floor

                first = True
                c1 = {}
                # All scores are the same for some reason, don't divide by zero
                if totalScore == 0.0:
                    totalScore = 1
                for c in sortedGeneration.itervalues():
                    if first:
                        c['probability'] = (c['maxScore'] - floor) / totalScore
                        first = False
                    else:
                        c['probability'] = (c['maxScore'] - floor) / totalScore + c1['probability']
                    c1 = c

            # How many of the best controllers are we keeping?
            numElites = popSize - (params['numberToMutate'] + params['numberOfChildren'])

            if (numElites < 0):
                raise NTRTMasterError("Population slated to grow in size! Please adjust population size, number to mutate and/or number of children")

            # Copy elites to new generation
            count = 0
            for c in sortedGeneration.itervalues():
                # Stop when number of elites has been reached
                if (count >= numElites):
                    break

                nextGeneration.__setitem__(c['paramID'], c)
                if c['params'] == None:
                    raise NTRTMasterError("Found it!")
                count += 1

            # Add 'asexual' mutations to next generation.
            # TODO: make option that assigns mutations to random controllers, rather than just mutating top N
            count = 0
            for c in sortedGeneration.itervalues():
                if (count >= params['numberToMutate']):
                    break
                cNew = {}
                cNew['params'] = self.__mutateParams(c['params'], paramName)
                cNew['paramID'] = str(self.paramID)
                cNew['scores'] = []

                nextGeneration.__setitem__(cNew['paramID'], cNew)
                self.paramID += 1

                if cNew['params'] == None:
                    raise NTRTMasterError("Found it!")

                count += 1

            # Add children to new generation
            for i in range(params['numberOfChildren']):

                c1Prob = random.random()
                c2Prob = random.random()

                c1 = self.__getControllerFromProbability(sortedGeneration, c1Prob)
                c2 = self.__getControllerFromProbability(sortedGeneration, c2Prob)

                while (c1 == c2):
                    c2Prob = random.random()
                    c2 = self.__getControllerFromProbability(sortedGeneration, c2Prob)

                cNew = {}
                cNew['params'] = self.__getChildController(c1['params'], c2['params'], params)

                if (random.random() >= params['childMutationChance']):
                    cNew['params'] = self.__mutateParams(cNew['params'], paramName)

                cNew['paramID'] = str(self.paramID)
                cNew['scores'] = []

                nextGeneration.__setitem__(cNew['paramID'], cNew)
                self.paramID += 1

                if cNew['params'] == None:
                    raise NTRTMasterError("Found it!")

            if (len(nextGeneration) != popSize):
                raise NTRTMasterError("Failed to generate the correct number of controllers")

            if (params['numberOfStates'] > 0):
                for c in nextGeneration.itervalues():
                    c['params']['neuralFilename'] = "logs/bestParameters-test_fb-"+ c['paramID'] +".nnw"
                    self.__writeToNNW(c['params']['neuralParams'], self.path + c['params']['neuralFilename'])

        return nextGeneration

    def getParamID(self, gen, jobNum):
        """
        Get the controller's key based on the job number
        """
        try:
            return list(gen)[jobNum]
        except IndexError:
            print 'Not enough keys'

    def getNewFile(self, jobNum):
        """
        Handle the generation of a new JSON file with new parameters. Will vary based on the
        learning method used and the config file
        Edit this based on your parameter set
        """

        obj = {}

        # Hacked co-evolution. Normally co-evolution would always select a random controller
        if(jobNum >= len(self.currentGeneration['node'])):
            jobNum_node = random.randint(0, len(self.currentGeneration['node']) - 1)
        else:
            jobNum_node = jobNum

        if(jobNum >= len(self.currentGeneration['edge'])):
            jobNum_edge = random.randint(0, len(self.currentGeneration['edge']) - 1)
        else:
            jobNum_edge = jobNum

        if(jobNum >= len(self.currentGeneration['feedback'])):
            jobNum_fb = random.randint(0, len(self.currentGeneration['feedback']) - 1)
        else:
            jobNum_fb = jobNum

        if(jobNum >= len(self.currentGeneration['goal'])):
            jobNum_goal = random.randint(0, len(self.currentGeneration['goal']) - 1)
        else:
            jobNum_goal = jobNum

        obj["nodeVals"] = self.currentGeneration['node'][self.getParamID(self.currentGeneration['node'], jobNum_node)]
        obj["edgeVals"] = self.currentGeneration['edge'][self.getParamID(self.currentGeneration['edge'], jobNum_edge)]
        obj["feedbackVals"] = self.currentGeneration['feedback'][self.getParamID(self.currentGeneration['feedback'], jobNum_fb)]
        obj["goalVals"] = self.currentGeneration['goal'][self.getParamID(self.currentGeneration['goal'], jobNum_goal)]

        outFile = self.path + self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

        fout = open(outFile, 'w')

        json.dump(obj, fout, indent=4)

        return self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

    def beginTrial(self):
        """
        Override this. It should just contain a loop where you keep constructing NTRTJobs, then calling
        runJob on it (which will block you until the NTRT instance returns), parsing the result from the job, then
        deciding if you should run another trial or if you want to terminate.
        """

        # Start a counter job ids to be use in dictionaries
        self.paramID = 1

        numTrials = self.jConf['learningParams']['numTrials']
        numGenerations = self.jConf['learningParams']['numGenerations']

        results = {}
        jobList = []
        self.currentGeneration = {}
        self.currentGeneration['edge'] = {}
        self.currentGeneration['node'] = {}
        self.currentGeneration['feedback'] = {}
        self.currentGeneration['goal'] = {}
        logFile = open('evoLog.txt', 'w') #Clear logfile
        logFile.close()

        scoreDump = open('scoreDump.txt', 'w')
        scoreDump.close()
        for n in range(numGenerations):
            # Create the generation
            self.currentGeneration['edge'] = self.generationGenerator(self.currentGeneration['edge'], 'edgeVals')
            self.currentGeneration['node'] = self.generationGenerator(self.currentGeneration['node'], 'nodeVals')
            self.currentGeneration['feedback'] = self.generationGenerator(self.currentGeneration['feedback'], 'feedbackVals')
            self.currentGeneration['goal'] = self.generationGenerator(self.currentGeneration['goal'], 'goalVals')

            # Iterate over the generation (change range..)
            if n > 0:
                startTrial = self.jConf['learningParams']['deterministic']
            else:
                startTrial = 0

            # We want to write all of the trials for post processing
            for i in range(0, numTrials) :

                # MonteCarlo solution. This function could be overridden with something that
                # provides a filename for a pre-existing file
                fileName = self.getNewFile(i)

                # All args to be passed to subprocess must be strings
                args = {'filename' : fileName,
                        'resourcePrefix' : self.jConf['resourcePath'],
                        'path'     : self.jConf['lowerPath'],
                        'executable' : self.jConf['executable'],
                        'length'   : self.jConf['learningParams']['trialLength'],
                        'terrain'  : self.jConf['terrain']}
                if (n == 0 or i >= startTrial):
                    jobList.append(BrianJob(args))

            # Run the jobs
            conSched = ConcurrentScheduler(jobList, self.numProcesses)
            completedJobs = conSched.processJobs()

            # Read scores from files, write to logs
            totalScore = 0
            maxScore = -1000
            for job in completedJobs:
                job.processJobOutput()
                jobVals = job.obj

                scores = jobVals['scores']

                edgeKey = jobVals ['edgeVals']['paramID']
                nodeKey = jobVals ['nodeVals']['paramID']
                feedbackKey = jobVals ['feedbackVals']['paramID']
                goalKey = jobVals ['goalVals']['paramID']

                lParams = self.jConf['learningParams']

                # Iterate through all of the new scores for this file
                for i in scores:
                    score = i['distance']

                    if (lParams['edgeVals']['learning']):
                        self.currentGeneration['edge'][edgeKey]['scores'].append(score)
                    if (lParams['nodeVals']['learning']):
                        self.currentGeneration['node'][nodeKey]['scores'].append(score)
                    if (lParams['feedbackVals']['learning']):
                        self.currentGeneration['feedback'][feedbackKey]['scores'].append(score)
                    if (lParams['goalVals']['learning']):
                        self.currentGeneration['goal'][goalKey]['scores'].append(score)
                    totalScore += score
                    if score > maxScore:
                        maxScore = score

            avgScore = totalScore / float(len(completedJobs) * len(self.jConf['terrain']) )
            logFile = open('evoLog.txt', 'a')
            logFile.write(str((n+1) * numTrials) + ',' + str(maxScore) + ',' + str(avgScore) +'\n')
            logFile.close()


class BrianJob(NTRTJob):

    def __init__(self, jobArgs):
        """
        Override this in your subclass. Be sure that at the end of your method your init method
        you make a call to self._setup(). I'll clean this up later so that we're properly doing a super
        call (rather than invoking setup in the child), no need for you to handle that now.

        You can put args into this however you want, just depends on what convention you want to use. I'd personally
        use a dictionary. If you use a dictionary, just use the jobArgs keyword from this function's signature.
        """
        logging.info("Constructing job with args %r" % jobArgs)
        self.args = jobArgs

        self._setup()

    def _setup(self):
        """
        This is where you'll handle setup related to this *single* learning trial. Each instance of NTRT
        we run will have its own NTRTJob instance.
        """

    def startJob(self):
        """
        Override this to start the NTRT instance and pass it the relevant parameters.. This is called
        by NTRTJobMaster when it wants to start this NTRT process.
        """

        logging.info("STARTING job with args %r" % self.args)
        self.pid = os.fork()

        if self.pid == 0:
            # Redirect the stdout output to dev null in the child.
            logPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename'] + '_log.txt'
            logFile = open(logPath, 'wb')

            # A set of jobs. Currently [0 0] is flat ground, [1 0] is a block field, [0 1] is hilly terrain, and [1 1] is both
            # This will expand in the future.
            terrainMatrix = self.args['terrain']
            # Update this if the subprocess call gets changed
            if len(terrainMatrix[0]) < 4:
                raise NTRTMasterError("Not enough terrain args!")

            # Run through a set of binary job options. Currently handles terrain switches
            for run in terrainMatrix:
                #TODO improve error handling here
                subprocess.check_call([self.args['executable'], "-l", self.args['filename'], "-s", str(self.args['length']), "-b", str(run[0]), "-H", str(run[1]), "-a", str(run[2]), "-B", str(run[3])], stdout=logFile)
            sys.exit()

    def processJobOutput(self):
        scoresPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename']

        try:
            fin = open(scoresPath, 'r')
            self.obj = json.load(fin)
            fin.close()
        except IOError:
            self.obj = {}

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    configFile = sys.argv[1]
    numProcesses = int(sys.argv[2])
    jobMaster = BrianJobMaster(configFile, numProcesses)
    jobMaster.beginTrial()

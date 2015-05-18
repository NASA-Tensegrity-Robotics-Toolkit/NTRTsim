import numpy as np
import sys
import os
import subprocess
import random
import json
import logging
from interfaces import NTRTJobMaster, NTRTMasterError
from NTRTJobMaster import MonteCarloJob
from concurrent_scheduler import ConcurrentScheduler

def sumOfSquares(x):

    total = sum(x[:]**2)
    print(total)
    return total

class SPSA(NTRTJobMaster):

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

        self.a0 = self.jConf['a0']
        self.c0 = self.jConf['c0']
        self.bp = self.jConf['bp']
        self.maxStep = self.jConf['maxStep']

        self.k = 1

        try:
            os.makedirs(self.path)
        except OSError:
            if not os.path.isdir(self.path):
                raise NTRTMasterError("Directed the folder path to an invalid address")

        # Consider seeding random, using default (system time) now
        #random.seed(5)

    def __getAk(self,k):

        return self.a0 / k

    def __getCk(self,k):
        return self.c0 / (k ** 0.125)

    def __getBernoulli(self, n):

        deltas = []

        for i in range(0, n):
            if random.random() > self.bp:
                deltas.append(0.1)
            else:
                deltas.append(-0.1)

        return np.array(deltas)

    def __getTestParams(self, x, deltas, ck, params):
        pMax = params['paramMax']
        pMin = params['paramMin']

        set1 = x + ck * deltas
        set2 = x - ck * deltas

        set1 = np.clip(set1, pMin, pMax)
        set2 = np.clip(set2, pMin, pMax)

        return [set1, set2]

    def __getGk(self, Ck, deltas, scores):
        return ((scores[0] - scores[1]) / (2.0 * Ck * deltas))

    def __updateParams(self, x, gk, ak, params):

        pMax = params['paramMax']
        pMin = params['paramMin']

        deltaX = -ak * gk

        newX = []

        for i in range(0, len(x)):
            newX.append(x[i] + np.sign(deltaX[i]) * min(abs(deltaX[i]), self.maxStep))

        nXArray = np.array(newX)

        nXArray = np.clip(nXArray, pMin, pMax)

        return nXArray

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

    def __arrayToJSON(self, params, xVals):

        newController = {}
        newController['paramID'] = str(self.paramID)

        if params['numberOfStates'] == 0 :

            newParams = []

            k = 0
            for i in range(0, params['numberOfInstances']) :
                    subParams = []
                    for j in range(0, params['numberOfOutputs']) :
                        # Assume scaling happens elsewhere
                        subParams.append(xVals[k])
                        k += 1
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

            print(totalParams)
            print(len(xVals))

            for i in range(0,  totalParams) :
                neuralParams.append(xVals[i])

            newParams['neuralParams'] = neuralParams

            self.__writeToNNW(neuralParams, self.path + newParams['neuralFilename'])

        newController['params'] = newParams
        newController['scores'] = []

        return newController

    def __JSONToArray(self, params, controller):

        if params['numberOfStates'] == 0 :
            xVals = []
            cVals = controller['params']

            for i in range(0, params['numberOfInstances']) :
                    for j in range(0, params['numberOfOutputs']) :
                        # Assume scaling happens elsewhere
                        xVals.append(cVals[i][j])

        else:
            xVals = controller['params']['neuralParams']

        return np.array(xVals)

    def generationGenerator(self, currentGeneration, paramName):
        """
        Master function that takes an existing set of paramters, sorts them by score
        and then returns a new set of parameters based on the specification file
        """

        params = self.jConf["learningParams"][paramName]

        useAvg = params['useAverage']

        nextGeneration = []

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
                    nextGeneration.append(controller)

                    self.paramID += 1

            # If no start seed, use random
            for i in range(len(nextGeneration), 1):
                if (i < 0):
                    raise NTRTMasterError("Number of controllers greater than population size!")

                controller = self.__getNewParams(paramName)
                nextGeneration.append(controller)
                self.paramID += 1

            newX = self.__JSONToArray(params, controller)

        elif (not params['learning']):
            # Not learning, return previous controllers
            nextGeneration = currentGeneration
            newX = np.array([0])

        else:
            # learning, not doing monteCarlo, have a previous generation
            if len(currentGeneration) != 3:
                raise NTRTMasterError("Incorrect input for this stage!")

            for controller in currentGeneration:
                try:
                    scores = controller['scores']
                except KeyError:
                    scores = [0]

                if len(scores) == 0:
                    scores = [0]
                print (scores)

                controller['maxScore'] = max(scores)
                controller['avgScore'] = sum(scores) / float(len(scores))

            oldX = self.__JSONToArray(params, currentGeneration[0])

            if (useAvg):
                newScores = [currentGeneration[1]['avgScore'], currentGeneration[2]['avgScore']]
            else:
                newScores = [currentGeneration[1]['maxScore'], currentGeneration[2]['maxScore']]

            gk = self.__getGk(self.__getCk(self.k), np.array(currentGeneration[0]['deltas']), newScores)

            newX = self.__updateParams(oldX, gk, self.__getAk(self.k), params)

            print("Generation Genorator")
            print(newX)

            newController = self.__arrayToJSON(params, newX)
            self.paramID += 1

            nextGeneration.append(newController)

        return [nextGeneration, newX]

    def nextTestParams(self, nextGeneration, xParams, paramName):

        print(paramName)
        print(xParams)


        params = self.jConf["learningParams"][paramName]

        deltas = self.__getBernoulli(len(xParams))

        # Store for future use
        nextGeneration[0]['deltas'] = deltas.tolist()

        print(self.__getCk(self.k))

        nextTests = self.__getTestParams(xParams, deltas, self.__getCk(self.k), params)

        print(nextTests)

        nextGeneration.append(self.__arrayToJSON(params, nextTests[0]))
        self.paramID += 1
        nextGeneration.append(self.__arrayToJSON(params, nextTests[1]))
        self.paramID += 1

        return nextGeneration

    def getNewFile(self, jobNum):
        """
        Handle the generation of a new JSON file with new parameters. Will vary based on the
        learning method used and the config file
        Edit this based on your parameter set
        """

        obj = {}

        # Hacked co-evolution. Normally co-evolution would always select a random controller
        for p in self.prefixes:
            if(jobNum >= len(self.currentGeneration[p])):
                jobNum_node = 0
            else:
                jobNum_node = jobNum

            obj[p + "Vals"] = self.currentGeneration[p][jobNum_node]


        outFile = self.path + self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

        fout = open(outFile, 'w')

        json.dump(obj, fout, indent=4)

        return self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

    def getJobNum(self, paramNum, paramName):

        for i in range(0, len(self.currentGeneration[paramName])):
            if paramNum == self.currentGeneration[paramName][i]['paramID']:
                break

        return i

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
        self.prefixes = ['edge', 'node', 'feedback', 'goal']

        results = {}
        jobList = []
        self.currentGeneration = {}

        for p in self.prefixes:
            self.currentGeneration[p] = {}

        logFile = open('evoLog.txt', 'w') #Clear logfile
        logFile.close()

        scoreDump = open('scoreDump.txt', 'w')
        scoreDump.close()

        lParams = self.jConf['learningParams']

        for n in range(numGenerations):
            # Create the generation
            for p in self.prefixes:
                [self.currentGeneration[p], xVals] = self.generationGenerator(self.currentGeneration[p], p + 'Vals')
                if (lParams[p + 'Vals']['learning']):
                    self.currentGeneration[p] = self.nextTestParams(self.currentGeneration[p], xVals, p + 'Vals')

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
                    jobList.append(MonteCarloJob(args))

            # Run the jobs
            conSched = ConcurrentScheduler(jobList, self.numProcesses)
            completedJobs = conSched.processJobs()

            # Read scores from files, write to logs
            totalScore = 0
            maxScore = -1000
            jobNum = startTrial
            for job in completedJobs:
                job.processJobOutput()
                jobVals = job.obj

                scores = jobVals['scores']

                jobNum =  self.getJobNum(jobVals['edgeVals']['paramID'], 'edge')

                # Iterate through all of the new scores for this file
                for i in scores:
                    score = i['distance']

                    for p in self.prefixes:
                        if (lParams[p + 'Vals']['learning']):
                            self.currentGeneration[p][jobNum]['scores'].append(score)

                    totalScore += score
                    if score > maxScore:
                        maxScore = score

                jobNum += 1

            avgScore = totalScore / float(len(completedJobs) * len(self.jConf['terrain']) )
            logFile = open('evoLog.txt', 'a')
            logFile.write(str((n+1) * numTrials) + ',' + str(maxScore) + ',' + str(avgScore) +'\n')
            logFile.close()

            self.k += 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    configFile = sys.argv[1]
    numProcesses = int(sys.argv[2])
    jobMaster = SPSA(configFile, numProcesses)
    jobMaster.beginTrial()

import numpy as np
import sys
import os
import subprocess
import random
import json
import logging
# TODO: Need to remove this namespace import and do a direct import of BrianJob (from wherever it ends up).
from interfaces import NTRTJobMaster, NTRTJob, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
from evolution import EvolutionJob

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
        self.A = self.jConf['A']
        self.maxStep = self.jConf['maxStep']

        self.k = 1

        try:
            os.makedirs(self.path)
        except OSError:
            if not os.path.isdir(self.path):
                raise NTRTMasterError("Directed the folder path to an invalid address")

        # Consider seeding random, using default (system time) now
        # random.seed(1)

    def __getAk(self,k):
        
        return self.a0 /(self.A + k)

    def __getCk(self,k):
        return self.c0 / (k ** 0.166)
        
    def __getBernoulli(self, n):

        deltas = []

        for i in range(0, n):
            if random.random() > self.bp:
                deltas.append(1)
            else:
                deltas.append(-1)
        
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
        
        deltaX = ak * gk
        
        newX = []

        for i in range(0, len(x)):
            #newX.append(x[i] + np.sign(deltaX[i]) * min(abs(deltaX[i]), self.maxStep))
            newX.append(x[i] + deltaX[i])
        
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

            #print(totalParams)
            #print(len(xVals))

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
    
    def evaluateNewController(self, currentGeneration, paramName):
        
        i = 0
        for controller in currentGeneration:
            try:
                scores = controller['scores']
                
                controller['maxScore'] = max(scores)
                controller['avgScore'] = sum(scores) / float(len(scores))
                
            except KeyError:
                
                self.runTrials(i, i)
                scores = controller['scores']
                
                controller['maxScore'] = max(scores)
                controller['avgScore'] = sum(scores) / float(len(scores))
            
            except ValueError:
                
                self.runTrials(i, i)
                scores = controller['scores']
                
                controller['maxScore'] = max(scores)
                controller['avgScore'] = sum(scores) / float(len(scores))
                
            finally:
                print (scores)

                i += 1
            
        
        params = self.jConf["learningParams"][paramName]

        useAvg = params['useAverage']
        
        if (useAvg):
            success = (currentGeneration[0]['avgScore'] < currentGeneration[1]['avgScore'])
            """
                       or (currentGeneration[0]['avgScore'] - 10.0 < currentGeneration[1]['avgScore']
                           and currentGeneration[0]['maxScore'] < currentGeneration[1]['maxScore']))
            """
        else:
            success = currentGeneration[0]['maxScore'] < currentGeneration[1]['maxScore']
        
        return success
    
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
            if len(currentGeneration) < 3:
                raise NTRTMasterError("Incorrect input for this stage! " + str(len(currentGeneration)))
            
            genSize = len(currentGeneration)
            
            print(paramName)
            
            i = 0
            for controller in currentGeneration:
                try:
                    scores = controller['scores']
                    
                    controller['maxScore'] = max(scores)
                    controller['avgScore'] = sum(scores) / float(len(scores))
                    
                except KeyError:
                    
                    self.runTrials(i, i)
                    scores = controller['scores']
                    
                    controller['maxScore'] = max(scores)
                    controller['avgScore'] = sum(scores) / float(len(scores))
                
                except ValueError:
                    
                    self.runTrials(i, i)
                    scores = controller['scores']
                    
                    controller['maxScore'] = max(scores)
                    controller['avgScore'] = sum(scores) / float(len(scores))
                    
                finally:
                    print (scores)

                    i += 1
            
            oldX = self.__JSONToArray(params, currentGeneration[genSize - 3])

            if (useAvg):
                newScores = [currentGeneration[genSize - 2]['avgScore'], currentGeneration[genSize - 1]['avgScore']]
            else:
                newScores = [currentGeneration[genSize - 2]['maxScore'], currentGeneration[genSize - 1]['maxScore']]

            gk = self.__getGk(self.__getCk(self.k), np.array(currentGeneration[genSize - 3]['deltas']), newScores)

            newX = self.__updateParams(oldX, gk, self.__getAk(self.k), params)

            # print(newX)

            newController = self.__arrayToJSON(params, newX)
            self.paramID += 1
            
            nextGeneration.append(currentGeneration[genSize - 3])
            nextGeneration.append(newController)
            
            logFile = open('bestScore.txt', 'a')
            logFile.write(str(self.k) + ',' + str(nextGeneration[0]['maxScore']) + ',' + str(nextGeneration[0]['avgScore']) +'\n')
            logFile.close()
            
        return [nextGeneration, newX]

    def nextTestParams(self, nextGeneration, xParams, paramName):

        print(paramName)
        #print(xParams)
        
        genSize = len(nextGeneration)
        
        print("At next test params " + str(genSize) + " " + str(self.k))
        try:
            print("Score " + str(nextGeneration[genSize - 1]["avgScore"]))
        except KeyError:
            print("No scores yet")
        
        params = self.jConf["learningParams"][paramName]

        deltas = self.__getBernoulli(len(xParams))

        # Store for future use
        nextGeneration[0]['deltas'] = deltas.tolist()
        if (len(nextGeneration) > 1):
            nextGeneration[1]['deltas'] = deltas.tolist()

        #print(self.__getCk(self.k))

        nextTests = self.__getTestParams(xParams, deltas, self.__getCk(self.k), params)

        #print(nextTests)

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

        for p in self.prefixes:
            if (self.lParams[p + 'Vals']['learning'] == False):
                #TODO This is a hackey solution to running mixed learning and non-learning sets. Update for the potential of non-learning sets
                jobNum_node = 0
            elif(jobNum >= len(self.currentGeneration[p])):
                jobNum_node = 0
                raise NTRTMasterError("Called a bad job number")
            
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
    
    def runTrials(self, st, nt):
        
        jobList = []
        
        print("Should run " + str(nt - st) + " jobs")
        
        numberToWrite = 0
        for p in self.prefixes:
            numberToWrite = max(numberToWrite, len(self.currentGeneration[p]))
        
        writeOut = min(numberToWrite, self.numTrials)
        
        if writeOut < nt - 1:
            raise NTRTMasterError("Not writing enough files")
        
        # We want to write all of the trials for post processing
        for i in range(0, writeOut) :

            # MonteCarlo solution. This function could be overridden with something that
            # provides a filename for a pre-existing file
            fileName = self.getNewFile(i)
            
            for j in self.jConf['terrain']:
                # All args to be passed to subprocess must be strings
                args = {'filename' : fileName,
                        'resourcePrefix' : self.jConf['resourcePath'],
                        'path'     : self.jConf['lowerPath'],
                        'executable' : self.jConf['executable'],
                        'length'   : self.jConf['learningParams']['trialLength'],
                        'terrain'  : j}
                if (i <= nt and i >= st):
                    jobList.append(EvolutionJob(args))
                    self.trialTotal += 1 

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

            # Iterate through all of the new scores for this file
            for i in scores:
                score = i['distance']

                for p in self.prefixes:
                    if (self.lParams[p + 'Vals']['learning']):
                        jobNum =  self.getJobNum(jobVals[p + 'Vals']['paramID'], p)
                        self.currentGeneration[p][jobNum]['scores'].append(score)

                totalScore += score
                if score > maxScore:
                    maxScore = score


        avgScore = totalScore / float(len(completedJobs) * len(self.jConf['terrain']) )
        logFile = open('evoLog.txt', 'a')
        logFile.write(str(self.trialTotal) + ',' + str(maxScore) + ',' + str(avgScore) +'\n')
        logFile.close()
    
    def beginTrial(self):
        """
        Override this. It should just contain a loop where you keep constructing NTRTJobs, then calling
        runJob on it (which will block you until the NTRT instance returns), parsing the result from the job, then
        deciding if you should run another trial or if you want to terminate.
        """

        # Start a counter job ids to be use in dictionaries
        self.paramID = 1
        self.trialTotal = 0

        self.numTrials = self.jConf['learningParams']['numTrials']
        numGenerations = self.jConf['learningParams']['numGenerations']
        
        self.lParams = self.jConf['learningParams']
        
        self.prefixes = []
        
        for keys in self.lParams:
            if keys[-4:] == "Vals":
                self.prefixes.append(keys[:-4])

        self.currentGeneration = {}
        self.oldGeneration = {}

        for p in self.prefixes:
            self.currentGeneration[p] = {}

        logFile = open('evoLog.txt', 'w') #Clear logfile
        logFile.close()
        
        logFile2 = open('bestScore.txt', 'w')
        logFile2.close()
        
        scoreDump = open('scoreDump.txt', 'w')
        scoreDump.close()
        
        xVals = {}

        for n in range(numGenerations):
            
            success = False
            
            for p in self.prefixes:
                self.oldGeneration[p] = self.currentGeneration[p]
            
            print("Old: " + str(len(self.oldGeneration[p])) + " New " + str(len(self.currentGeneration[p])))
            
            # Create the generation
            print("New Generation!")
            for p in self.prefixes:
                [self.currentGeneration[p], xVals[p]] = self.generationGenerator(self.currentGeneration[p], p + 'Vals')
            
            if n > 0:
                
                # Zero should already have been evaluated
                self.runTrials(1, 1)
                
                for p in self.prefixes:
                    
                    if (self.lParams[p + 'Vals']['learning']):
                    
                        success = self.evaluateNewController(self.currentGeneration[p],  p + 'Vals')
                        
                        if success == False:
                            print ("Backtracking! " + str(len(self.oldGeneration[p])) + " Run " + str(n))
                            
                            self.currentGeneration[p][1] = self.currentGeneration[p][0]
                            
                            # Only want to backtrack once
                            xVals[p] = self.__JSONToArray(self.lParams[p + 'Vals'], self.currentGeneration[p][1])
            
            genSize = 0
            
            for p in self.prefixes:
                if (self.lParams[p + 'Vals']['learning']):
                        self.currentGeneration[p] = self.nextTestParams(self.currentGeneration[p], xVals[p], p + 'Vals')
                        genSize = max(genSize, len(self.currentGeneration[p]))
                        
            print("Post run " + str(len(self.currentGeneration[p])))
            
            
            if n > 0:
                self.runTrials(genSize - 2, genSize - 1)
            else:
                self.runTrials(0, genSize - 1)

            self.k += 1
            

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    configFile = sys.argv[1]
    numProcesses = int(sys.argv[2])
    jobMaster = SPSA(configFile, numProcesses)
    jobMaster.beginTrial()

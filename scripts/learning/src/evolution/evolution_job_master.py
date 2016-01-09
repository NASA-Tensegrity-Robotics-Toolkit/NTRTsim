import os
import json
import random
import collections
from interfaces import NTRTJobMaster, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
import collections
#TODO: This is hackety, fix it.
from evolution_job import EvolutionJob

class LastUpdatedOrderedDict(collections.OrderedDict):
    'Store items in the order the keys were last added'

    def __setitem__(self, key, value):
        if key in self:
            del self[key]
        collections.OrderedDict.__setitem__(self, key, value)


class EvolutionJobMaster(NTRTJobMaster):
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
        
        try:
            os.makedirs(self.path + '/logs')
        except OSError:
            if not os.path.isdir(self.path + '/logs'):
                raise NTRTMasterError("Please create logs directory at" + self.path)

        
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

	numTrials = self.jConf['learningParams']['numTrials']

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
		if(params['monteCarlo'] and params['populationSize'] != numTrials):
	            raise NTRTMasterError("Number of trials must equal population size!")
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

        for p in self.prefixes:
            # Hacked co-evolution. Normally co-evolution would always select a random controller
            if(jobNum >= len(self.currentGeneration[p])):
                paramNum = random.randint(0, len(self.currentGeneration[p]) - 1)
            else:
                paramNum = jobNum
            
            obj[p + "Vals"] = self.currentGeneration[p][self.getParamID(self.currentGeneration[p], paramNum)]

	obj["metrics"] = [] # Added to store tension and COM data. 
        
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

        results = {}
        jobList = []
        
        lParams = self.jConf['learningParams']
        
        self.prefixes = []
        
        for keys in lParams:
            if keys[-4:] == "Vals":
                self.prefixes.append(keys[:-4])
        
        print (self.prefixes)
        
        self.currentGeneration = {}
        for p in self.prefixes:
            self.currentGeneration[p] = {}

        logFile = open('evoLog.txt', 'w') #Clear logfile
        logFile.close()

        scoreDump = open('scoreDump.txt', 'w')
        scoreDump.close()
        for n in range(numGenerations):
            # Create the generation'
            for p in self.prefixes:
                self.currentGeneration[p] = self.generationGenerator(self.currentGeneration[p], p + 'Vals')

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
                
                for j in self.jConf['terrain']:
                    # All args to be passed to subprocess must be strings
                                  
                    args = {'filename' : fileName,
                            'resourcePrefix' : self.jConf['resourcePath'],
                            'path'     : self.jConf['lowerPath'],
                            'executable' : self.jConf['executable'],
                            'length'   : self.jConf['learningParams']['trialLength'],
                            'terrain'  : j}
                    if (n == 0 or i >= startTrial):
                        jobList.append(EvolutionJob(args))

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
                        if (lParams[p + 'Vals']['learning']):
                            key = jobVals [p + 'Vals']['paramID']
                            self.currentGeneration[p][key]['scores'].append(score)

                    totalScore += score
                    if score > maxScore:
                        maxScore = score

            avgScore = totalScore / float(len(completedJobs) * len(self.jConf['terrain']) )
            logFile = open('evoLog.txt', 'a')
            logFile.write(str((n+1) * numTrials) + ',' + str(maxScore) + ',' + str(avgScore) +'\n')
            logFile.close()


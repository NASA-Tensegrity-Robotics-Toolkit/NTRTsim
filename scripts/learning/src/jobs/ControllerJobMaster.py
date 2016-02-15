import logging
import json
import random

from algorithms import dispatchLearning
from helpersNew import Generation
from helpersNew import Controller
from helpersNew import dictTools
from concurrent_scheduler import ConcurrentScheduler
from LearningJobMaster import LearningJobMaster
#TODO: This is hackety, fix it.
from evolution import EvolutionJob

class ControllerJobMaster(LearningJobMaster):


    """
    Component = the actual dictionary
    Component Name = the string name of the component

    For MonteCarlo, generationSize is king for how many members are made
    You can theoretically have a smaller generation than pop size for each MC
    """

    NEURAL_NET_DIRECTORY_NAME = "NeuralNet/"

    def _setup(self):

        logging.getLogger("NTRT Learning")
        super(ControllerJobMaster, self)._setup()

        # Create directory for NN files
        neuralNetDirectory = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME
        dictTools.tryMakeDir(neuralNetDirectory)

    def generationGenerator(self, previousGeneration):
        componentsConfig = self.getComponentsConfig()
        # Copies the componentPopulations from previous and increments the generationID
        componentPopulations = previousGeneration.getComponentPopulations()
        # This is a hack for backwards compatibility
        # Fix up later
        generationID = previousGeneration.getID() + 1

        nextGeneration = Generation(generationID)
        for componentName in componentsConfig:
            population = self.generateComponentPopulation(componentName,
                                                          componentsConfig,
                                                          componentPopulations[componentName],
                                                          generationID)
            nextGeneration.addComponentPopulation(componentName, population)

        generationSize = self.config['TrialProperties']['generationSize']
        for i in range(generationSize):
            # this generator might not belong here
            nextGeneration.generateMemberFromComponents()

        # Perform generation creation
        # Need a better name for this
        # nextGeneration = self.createNewGeneration(componentPopulations)
        return nextGeneration

    # Once backwards compatibility is dealt with, this can be moved up to LearningJobMaster
    # componentName does not need to be passed here
    # just get it earlier from self.getComponentsConfig()[componentName]
    def generateComponentPopulation(self, componentName, componentsConfig, componentPopulation, generationID):
        #print "genID in generateComponentPopulation: " + str(generationID)
        # Note singular from plural
        componentConfig = componentsConfig[componentName]
        emptyComponent = self.createEmptyComponent(componentsConfig[componentName]['NeuralNetwork'])
        newComponentPopulation = dispatchLearning(componentConfig=componentConfig,
                                                  scoreMethod=self.config["TrialProperties"]["scoreMethod"],
                                                  fitnessFunction=self.config["TrialProperties"]["fitnessFunction"],
                                                  baseComponent=emptyComponent,
                                                  componentPopulation=componentPopulation
                                                  )

        # Move this to its own function when possible
        # Backwards Compatibility
        # Metrics is not added because it is never seen to be used
        # Move this to its own function afterwards
        for componentMember in newComponentPopulation:
            # Index the component in the line of all of these components made so far in learning
            # Consider skipping this
            populationSize = self.config[componentName]['PopulationSize']
            componentMember['paramID'] = populationSize * generationID + componentMember['populationID']
            componentMember['scores'] = []

            ### Hacky hacky hacky
            ### HACKY HACKY HACKY
            # Backward Compatibility
            if "numHidden" in componentMember['params']:
                baseFileName = self.config['PathInfo']['fileName'] + "_" + componentName + "_" + \
                               str(generationID) + "_" + str(componentMember['populationID']) + '.nnw'

                fileName = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME + baseFileName
                self.writeToNNW(componentMember['params']['neuralParams'], fileName)
                componentMember['params']['neuralFilename'] = self.NEURAL_NET_DIRECTORY_NAME + baseFileName

        return newComponentPopulation

    """
    This is taken straight from Brian's code.
    Clean up later before major commit
    This does not belong here.
    Related: Why is the NNW file path being stored at all?
             It should be synthesized from info in the dictionary.
    """
    # Backwards Compatibility
    # Something about this function doesn't belong
    def writeToNNW(self, neuralParams, fileName):
        fout = open(fileName, 'w')
        first = True
        for x in neuralParams:
            if (first):
                fout.write(str(x))
                first = False
            else:
                fout.write("," + str(x))

    def createEmptyComponent(self, neuralNet):
        # print "Current component: " + component
        if neuralNet['numberOfStates'] == 0:
            emptyComponent = self.getNonNNParams(neuralNet)
        else:
            emptyComponent = self.getNNParams(neuralNet)
        # dictTools.printDict(emptyComponent)
        return emptyComponent

    def getNonNNParams(self, neuralNet):
        numInstances = neuralNet['numberOfInstances']
        numOutputs = neuralNet['numberOfOutputs']
        params = []

        for i in range(numInstances):
            subParams = []
            for j in range(numOutputs):
                subParams.append(None)
            params.append(subParams)

        return {'params' : params}

    def getNNParams(self, neuralNet):
        numOutputs = neuralNet['numberOfOutputs']
        numStates = neuralNet['numberOfStates']
        numHidden = neuralNet['numberOfHidden']
        totalParams = (numStates + 1) * (numHidden) + (numHidden + 1) * numOutputs

        newNeuro = {
            'neuralParams' : [None] * totalParams,
            'numActions'   : numOutputs,
            'numStates'    : numStates,
            'numHidden'    : numHidden
            #'neuralFilename' : "logs/bestParameters-test_fb-"+ newController['paramID'] +".nnw"}
        }
        return {'params' : newNeuro}

    # Need to set up a call system to this from child jobs
    # Maybe pass it a function of type generation -> generation?
    def beginTrial(self):
        self.beginTrialMaster(generationGeneratorFuction=self.generationGenerator)

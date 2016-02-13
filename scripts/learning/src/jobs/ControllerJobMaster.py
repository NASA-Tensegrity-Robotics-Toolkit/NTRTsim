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
        componentsDictionary = self.getComponentsDictionary()
        generationID = self.getNewGenerationID(previousGeneration)

        logging.info("Starting algorithms")
        componentPopulations = self.generateComponentPopulations(componentsDictionary, previousGeneration)
        # Perform generation creation
        # Need a better name for this
        newGeneration = self.createNewGeneration(componentPopulations, generationID)

        return newGeneration

    def generateComponentPopulations(self, componentsDictionary, previousGeneration):
        #print "genID in generateComponentPopulations: " + str(generationID)
        componentPopulations = {}
        for componentName in componentsDictionary:
            population = self.generateComponentPopulation(componentName,
                                                          componentsDictionary,
                                                          previousGeneration)
            componentPopulations[componentName] = population
        return componentPopulations

    # "Generation" is explicitly the generation object
    # "Population" is just a list or dictionary
    def generateComponentPopulation(self, componentName, componentsDictionary, previousGeneration):
        #print "genID in generateComponentPopulation: " + str(generationID)
        generationID = self.getNewGenerationID(previousGeneration)
        # Note singular from plural
        componentDictionary = componentsDictionary[componentName]
        emptyComponent = self.createEmptyComponent(componentName, componentsDictionary[componentName]['NeuralNetwork'], generationID)
        componentPopulation = dispatchLearning(componentName=componentName,
                                            componentDictionary=componentDictionary,
                                            baseComponent=emptyComponent,
                                            previousGeneration=previousGeneration
                                            )

        # Move this to its own function when possible
        # Backward Compatibility
        # Metrics is not added because it is never seen to be used
        for component in componentPopulation:
            # Index the component in the line of all of these components made so far in learning
            populationSize = self.config[componentName]['PopulationSize']
            component['paramID'] = populationSize * generationID + component['populationID']
            component['scores'] = []

            ### Hacky hacky hacky
            ### HACKY HACKY HACKY
            # Backward Compatibility
            if "numHidden" in component['params']:
                baseFileName = self.config['PathInfo']['fileName'] + "_" + componentName + "_" + str(component['generationID']) + "_" + str(component['populationID']) + '.nnw'
                fileName = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME + baseFileName
                self.writeToNNW(component['params']['neuralParams'], fileName)
                component['params']['neuralFilename'] = self.NEURAL_NET_DIRECTORY_NAME + baseFileName

        return componentPopulation

    """
    This is taken straight from Brian's code.
    Clean up later before major commit
    This does not belong here.
    Related: Why is the NNW file path being stored at all?
             It should be synthesized from info in the dictionary.
    """
    # Backward Compatibility
    def writeToNNW(self, neuralParams, fileName):
        fout = open(fileName, 'w')
        first = True
        for x in neuralParams:
            if (first):
                fout.write(str(x))
                first = False
            else:
                fout.write("," + str(x))

    def createEmptyComponent(self, component, neuralNet, generationID):
        emptyComponent = None
        # print "Current component: " + component
        if neuralNet['numberOfStates'] == 0:
            emptyComponent = self.getNonNNParams(neuralNet)
        else:
            emptyComponent = self.getNNParams(neuralNet)
        # dictTools.printDict(emptyComponent)
        emptyComponent['generationID'] = generationID
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

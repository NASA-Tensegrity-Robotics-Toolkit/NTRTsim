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
        components = self.getComponents()
        generationID = getNewGenerationID(previousGeneration)

        logging.info("Starting algorithms")
        componentPopulations = self.generateComponentPopulations(components, previousGeneration, generationID)
        # Perform generation creation
        newGeneration = self.createNewGeneration(componentPopulations, generationID)

        return newGeneration

    def getComponents(self):
        components = {}
        for key in self.config:
            if key == 'PathInfo' or key == 'TrialProperties':
                pass
            else:
                components[key] = self.config[key]
        return components

    # Better to pass the generation object instead of doing this
    # If not used by commit, delete this
    # Where is self.previousGeneration coming from?
    def getPreviousComponentGeneration(self, component, previousGeneration=None):
        previousComponentGeneration = []
        if len(previousGeneration.getMembers()) > 0:
            for controller in previousGeneration.getMembers():
                previousComponent = controller.components[component]
                previousComponentGeneration.append(previousComponent)
        return previousComponentGeneration

    def generateComponentPopulations(self, components, previousGeneration, generationID):
        #print "genID in generateComponentPopulations: " + str(generationID)
        componentPopulations = {}
        for componentName in components:
            population = self.generateComponentPopulation(componentName, components, previousGeneration, generationID)
            componentPopulations[componentName] = population
        return componentPopulations

    # "Generation" is expliclty the generation object
    # "Population" is just a list or dictionary
    def generateComponentPopulation(self, componentName, components, previousGeneration, generationID):
        #print "genID in generateComponentPopulation: " + str(generationID)
        componentDictionary = components[componentName]
        emptyComponent = self.createEmptyComponent(componentName, components[componentName]['NeuralNetwork'], generationID)
        componentPopulation = dispatchLearning(componentName=componentName,
                                            templateComponent=emptyComponent,
                                            componentDictionary=componentDictionary,
                                            previousGeneration=previousGeneration
                                            )
        ### Make component backwards compatible
        ### Move this to its own function when possible
        ### Metrics is not added because it is never seen to be used
        for component in componentPopulation:
            # Index the component in the line of all of these components made so far in learning
            populationSize = self.config[componentName]['PopulationSize']
            component['paramID'] = populationSize * generationID + component['populationID']
            component['scores'] = []

            ### Hacky hacky hacky
            ### HACKY HACKY HACKY
            if "numHidden" in component['params']:
                baseFileName = self.config['PathInfo']['fileName'] + "_" + componentName + "_" + str(component['generationID']) + "_" + str(component['populationID']) + '.nnw'
                fileName = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME + baseFileName
                self.writeToNNW(component['params']['neuralParams'], fileName)
                component['params']['neuralFilename'] = self.NEURAL_NET_DIRECTORY_NAME + baseFileName

        return componentPopulation

    """
    This is taken straight from Brian's code.
    Clean up later before major commit
    """
    def writeToNNW(self, neuralParams, fileName):
        fout = open(fileName, 'w')
        first = True
        for x in neuralParams:
            if (first):
                fout.write(str(x))
                first = False
            else:
                fout.write("," + str(x))


    # ID is id of previousGeneration + 1
    # if not previousGeneration then ID = 0
    def createNewGeneration(self, componentPopulations, generationID):
        # Running into issues with instantiating new controller objects
        # Will probably have the same issue here with Generation objects
        newGeneration = Generation(generationID)
        # print "genID in createNewGeneration: " + str(generationID)
        # Not really using id here...
        for id in range(self.config['TrialProperties']['generationSize']):
            #print "newController memID & genID: " + str(id) +" "+str(generationID)
            newController = Controller(memberID=id,generationID=generationID)
            #print "newController memID & genID: " + str(newController.components['memberID']) +" "+str(newController.components['generationID'])
            #print "size of controller.components: " + str(len(newController.components))
            #print "--"
            #dictTools.pause()
            if "edgeVals" in newController.components:
                dictTools.printDict(newController.components['edgeVals'])
            for component, population in componentPopulations.iteritems():
                randomPopulation = random.choice(population)
                newController.components[component] = randomPopulation
            #dictTools.printDict(newController.components['edgeVals'])
            newGeneration.addMember(newController)

        return newGeneration

    # Empty controllers are created as a form of error checking
    # If, by the end of generation, there are any "None" values,
    # something has been missed
    # UNUSED
    def createEmptyController(self, components):
        emptyController = Controller()
        for component in components:
            neuralNet = components[component]['NeuralNetwork']
            # print "Current component: " + component
            if neuralNet['numberOfStates'] == 0:
                emptyController.components[component] = self.getNonNNParams(neuralNet)
            else:
                emptyController.components[component] = self.getNNParams(neuralNet)
        return emptyController

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

    def importControllerSeedMembers(self):
        seedMembers = self.importSeedMembers()
        # Can return an empty previous generation
        seedControllerMembers = [Controller(seedMember=member) for member in seedMembers]
        previousGeneration = Generation(-1)
        for controller in seedControllerMembers:
            previousGeneration.addMember(controller)
        return previousGeneration

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

    # This should be moved to the member class
    def writeControllerToFile(self, controller):
        components = controller.components
        #dictTools.printDict(components)
        #dictTools.pause()
        #print self.config['PathInfo']['fileName']
        for key in components:
            print key
        # print components['generationID']
        # print components['memberID']
        # dictTools.pause()
        basename = self.config['PathInfo']['fileName'] + "_" + str(components['generationID']) + "_" + str(components['memberID']) + ".json"
        filePath = self.trialDirectory + '/' + basename
        # print "writing file to: " + filePath
        jsonFile = open(filePath, 'w')
        json.dump(components, jsonFile, indent=4)
        jsonFile.close()
        # dictTools.pause("Check that file was created.")
        return basename

    # Need to set up a call system to this from child jobs
    # Maybe pass it a function of type generation -> generation?
    def beginTrial(self):

        """
        Override this. It should just contain a loop where you keep constructing NTRTJobs, then calling
        runJob on it (which will block you until the NTRT instance returns), parsing the result from the job, then
        deciding if you should run another trial or if you want to terminate.
        """

        # Start a counter job ids to be use in dictionaries
        self.paramID = 1

        generationCount = self.trialProperties['generationCount']
        populationSize = self.trialProperties['generationSize']

        jobList = []

        lParams = self.trialProperties

        previousGeneration = self.importControllerSeedMembers()
        for genNum in range(generationCount):
            # We want to write all of the trials for post processing
            activeGeneration = self.generationGenerator(previousGeneration)

            # for member in activeGeneration.getMembers():
            index = 0

            for member in activeGeneration.getMembers():

                # Expecting a json-compatible dictionary
                fileName = self.writeControllerToFile(member)
                # dictTools.pause("fileName: " + fileName)

                for terrain in self.trialProperties['terrains']:
                    # Default to flat terrain for now
                    terrain = [[0, 0, 0, 0]]
                    # All args to be passed to subprocess must be strings
                    # TODO check if these args need to be passed from the yamlconfig


                    """
                    args = {'filename' : fileName,
                            'resourcePrefix' : self.jConf['resourcePath'],
                            'path'     : self.jConf['lowerPath'],
                            'executable' : self.jConf['executable'],
                            'length'   : self.jConf['learningParams']['trialLength'],
                            'terrain'  : j}
                    if (n == 0 or i >= startTrial):
                    """

                    args = {'filename' : fileName,
                            'resourcePrefix' : self.RESOURCE_DIRECTORY_NAME,
                            'path'     : self.config['PathInfo']['trialPath'],
                            'executable' : self.config['PathInfo']['executable'],
                            'length'   : self.trialProperties['trialLength'],
                            'terrain'  : terrain}
                    jobList.append(EvolutionJob(args))

                index += 1

            # Run the jobs
            conSched = ConcurrentScheduler(jobList, self.numProcesses)
            completedJobs = conSched.processJobs()

            for job in completedJobs:
                job.processJobOutput()

            previousGeneration = activeGeneration


def getNewGenerationID(previousGeneration):
    newID = 0
    if previousGeneration:
        newID = previousGeneration.ID + 1
    return newID

"""
    def temp(self):
        for keys in lParams:
            if keys[-4:] == "Vals":
                self.prefixes.append(keys[:-4])

        print (self.prefixes)

        self.currentGeneration = {}
        for p in self.prefixes:
            self.currentGeneration[p] = {}

        for n in range(generationCount):
            # Create the generation'
            for p in self.prefixes:
                self.currentGeneration[p] = self.generationGenerator(self.currentGeneration[p], p + 'Vals')

        return super(ControllerJobMaster, self).generationGenerator()
"""

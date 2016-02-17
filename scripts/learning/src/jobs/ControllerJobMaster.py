from helpersNew import Generation
from helpersNew import dictTools
from LearningJobMaster import LearningJobMaster

class ControllerJobMaster(LearningJobMaster):

    NEURAL_NET_DIRECTORY_NAME = "NeuralNet/"

    def _setup(self):
        super(ControllerJobMaster, self)._setup()
        # Create directory for NN files
        neuralNetDirectory = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME
        dictTools.tryMakeDir(neuralNetDirectory)

    def getComponentsKey(self):
        return None

    """
    Once the nnw file write is moved out of learning, the functions below dissapear.
    As a result, learning jobs are reduced to just "getComponentsKey"
    """

    def generationGenerator(self, previousGeneration):
        componentsConfig = self.getComponentsConfig()
        componentPopulations = previousGeneration.getComponentPopulations()
        generationID = previousGeneration.getID() + 1

        nextGeneration = Generation(generationID)
        for componentName in componentsConfig:
            population = self.generateComponentPopulation(componentsConfig[componentName],
                                                          componentPopulations[componentName])
            self.formatComponentPopulation(componentName, generationID,population)
            nextGeneration.addComponentPopulation(componentName, population)

        generationSize = self.config['TrialProperties']['generationSize']
        for i in range(generationSize):
            nextGeneration.generateMemberFromComponents()

        return nextGeneration

    def formatComponentPopulation(self, componentName, generationID, componentPopulation):
        for component in componentPopulation:
            self.formatComponent(component, generationID, componentName)

    def formatComponent(self, component, generationID, componentName):
        if "numHidden" in component['params']:
            baseFileName = self.config['PathInfo']['fileName'] + "_" + componentName + "_" + \
                           str(generationID) + "_" + str(component['populationID']) + '.nnw'

            fileName = self.trialDirectory + '/' + self.NEURAL_NET_DIRECTORY_NAME + baseFileName
            self.writeToNNW(component['params']['neuralParams'], fileName)
            component['params']['neuralFilename'] = self.NEURAL_NET_DIRECTORY_NAME + baseFileName

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
        fout.close()

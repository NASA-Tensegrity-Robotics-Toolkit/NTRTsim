import os
import yaml
import logging
from interfaces import NTRTJobMaster, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
from algorithms import dispatchLearning
from LearningJob import LearningJob
from helpersNew import Generation
from helpersNew import dictTools

class LearningJobMaster(NTRTJobMaster):

    RESOURCE_DIRECTORY_NAME    = "../../../resources/src/"

    # File Names
    LOG_FILE_NAME = "output.log"
    LOGGING_NAME = "NTRT Learning"
    SUMMARY_FILE_NAME = "summary.txt"

    LEARNING_CONFIG_KEYWORDS = [
        "PathInfo",
        "TrialProperties",
    ]

    PROTECTED_TERMS = [
        "generationID",
        "scores",
        "memberID",
        "trialPath",
        "fileName",
        "executable",
        "seedDirectory",
        "learningType",
        "triallength",
        "generationCount",
        "generationSize",
        "scoreMethod",
        "fitnessFunction",
        "terrains",
        "Algorithms",
        "PopulationSize",
        "Ranges"
    ]

    TERRAINS = {
        "flat" : [[0, 0, 0, 0]],
        "hill" : [[0, 0, 0, 0.0, 60000]]
    }

    def _setup(self):

        # If this fails, the program should fail. Input file is required.
        try:
            configFile = open(self.configFileName, 'r')
            self.config = yaml.load(configFile)
            configFile.close()
        except IOError:
            raise NTRTMasterError("Please provide a valid configuration file")

        self.trialDirectory = os.path.abspath(self.RESOURCE_DIRECTORY_NAME + self.config['PathInfo']['trialPath'])
        dictTools.tryMakeDir(self.trialDirectory)

        self.logFileName = self.trialDirectory + '/' + self.LOG_FILE_NAME
        logging.getLogger("NTRT Learning")
        logging.basicConfig(filename=self.logFileName, format='%(asctime)s:%(levelname)s:%(message)s', level=logging.DEBUG)

        # Backwards compatibility
        terrains = []
        for terrain in self.config['TrialProperties']['terrains']:
            try:
                terrains.append(self.TERRAINS[terrain])
            except Exception:
                raise Exception("Could not find terrain of type " + terrain + " in terrainMap.")

        self.config['TrialProperties']['terrains'] = terrains
        self.trialProperties = self.config['TrialProperties']

    memberTemplate = None
    def getMemberTemplate(self):
        if not self.memberTemplate:
            seedDirectory = self.config['PathInfo']['seedDirectory']
            try:
                templateFilePath = seedDirectory + os.listdir(seedDirectory)[0]
            except Exception:
                raise Exception("Seed Directory is empty. At least one seed member is needed to run a trial.")
            print templateFilePath
            self.memberTemplate = dictTools.loadFile(templateFilePath)
        return self.memberTemplate

    def getComponentsKey(self):
        return NotImplementedError("ComponentsPath must be implemented in subclass.")

    def getComponentsConfig(self):
        componentsConfig = {}
        componentsKey = self.getComponentsKey()
        componentsConfigDict = self.config
        if componentsKey:
            componentsConfigDict = self.config[componentsKey]
        for key in componentsConfigDict:
            if not key in self.LEARNING_CONFIG_KEYWORDS:
                componentsConfig[key] = componentsConfigDict[key]
        return componentsConfig

    # TODO: Consider a better name for this function. There is a Member class which makes this confusing.
    def getMemberFileType(self):
        seedDirectory = self.config['PathInfo']['seedDirectory']
        templateFilePath = seedDirectory + os.listdir(seedDirectory)[0]
        filename, fileExtension = os.path.splitext(templateFilePath)
        return fileExtension

    def writeMemberToFile(self, member):
        memberTemplate = self.getMemberTemplate()
        componentsKey = self.getComponentsKey()
        components = member.components
        if componentsKey:
            memberTemplate[componentsKey] = components
        else:
            memberTemplate = components
        basename = self.config['PathInfo']['fileName'] + "_" + str(components['generationID']) \
                   + "_" + str(components['memberID']) + self.getMemberFileType()
        filePath = self.trialDirectory + '/' + basename
        # TODO: dictTools is becoming a longterm helper tool. It needs the "Pretty Woman" treatment.
        dictTools.dumpFile(memberTemplate, filePath)
        return basename

    def generateComponentPopulation(self, componentConfig, componentPopulation):
        trialProperties = self.config["TrialProperties"]
        newComponentPopulation = dispatchLearning(componentConfig=componentConfig,
                                                  scoreMethod=trialProperties["scoreMethod"],
                                                  fitnessFunction=trialProperties["fitnessFunction"],
                                                  componentPopulation=componentPopulation
                                                  )
        return newComponentPopulation

    def generationGenerator(self, previousGeneration):
        componentsConfig = self.getComponentsConfig()
        componentPopulations = previousGeneration.getComponentPopulations()
        generationID = previousGeneration.getID() + 1
        nextGeneration = Generation(generationID)

        for componentName in componentsConfig:
            population = self.generateComponentPopulation(componentsConfig[componentName],
                                                          componentPopulations[componentName])
            nextGeneration.addComponentPopulation(componentName, population)

        generationSize = self.config['TrialProperties']['generationSize']
        for i in range(generationSize):
            nextGeneration.generateMemberFromComponents()

        return nextGeneration

    def importSeedGeneration(self):
        seedDirectory = self.config['PathInfo']['seedDirectory']
        previousGeneration = Generation(-1)
        if os.path.isdir(seedDirectory):
            for file in os.listdir(seedDirectory):
                absFilePath = os.path.abspath(seedDirectory) + '/' + file
                seedInput = dictTools.loadFile(absFilePath)
                componentsKey = self.getComponentsKey()
                if componentsKey:
                    seedInput = seedInput[componentsKey]
                for componentName, component in seedInput.iteritems():
                    if componentName not in self.PROTECTED_TERMS:
                        previousGeneration.addComponentMember(componentName, component)
        else:
            raise Exception("Trying to import from a seed directory that is not a seed directory. "
                            "Check the seedDirectory element in PathInfo.")
        return previousGeneration

    def beginTrial(self):

        jobList = []
        previousGeneration = self.importSeedGeneration()
        generationCount = self.trialProperties['generationCount']

        for genNum in range(generationCount):
            activeGeneration = self.generationGenerator(previousGeneration)

            for member in activeGeneration.getMembers():
                fileName = self.writeMemberToFile(member)

                for terrain in self.trialProperties['terrains']:
                    args = {'filename' : fileName,
                            'resourcePrefix' : self.RESOURCE_DIRECTORY_NAME,
                            'path'     : self.config['PathInfo']['trialPath'],
                            'executable' : self.config['PathInfo']['executable'],
                            'length'   : self.trialProperties['trialLength'],
                            'terrain'  : terrain}
                    # Pass member by reference for updating scores
                    jobList.append(LearningJob(args, member))

            # Only controller simulation is implemented currently
            if self.trialProperties['learningType'] == "controller":
                conSched = ConcurrentScheduler(jobList, self.numProcesses)
                completedJobs = conSched.processJobs()

            # for job in completedJobs:
            #     job.processJobOutput()
            #     scores = job.obj['scores']
            #     paramID = job.obj['paramID']
            #     activeGeneration.getMembers(paramID)

            previousGeneration = activeGeneration

import os
import yaml
import logging
from interfaces import NTRTJobMaster, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
from algorithms import dispatchLearning
#TODO: This is hackety, fix it.
from LearningJob import LearningJob
from helpersNew import Generation
from helpersNew import Member
from helpersNew import dictTools

class LearningJobMaster(NTRTJobMaster):

    ## Refactoring out directory names, using a flat file system
    # Directory Names
    RESOURCE_DIRECTORY_NAME    = "../../../resources/src/"
    GENERATIONS_DIRECTORY_NAME = "Generations/"
    MEMBERS_DIRECTORY_NAME     = "Members/"
    TRIALS_DIRECTORY_NAME      = "Trials/"
    # Rename this
    # GENERAL_DIRECTORY_NAME = "OutputMembers/"

    # File Names
    LOG_FILE_NAME              = "output.log"
    LOGGING_NAME               = "NTRT Learning"
    SUMMARY_FILE_NAME          = "summary.txt"

    LEARNING_CONFIG_KEYWORDS = [
        "PathInfo",
        "TrialProperties",
    ]

    # Protected Words
    # These are words used in defining basic learning properties in YAML
    PROTECTED_TERMS = [
        # These are to strip fields from seed members
        "generationID",
        "scores",
        "memberID",
        # These are added for filtering learning config files
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

        # If this fails, the program should fail. Input file is required
        # for useful output
        try:
            configFile = open(self.configFileName, 'r')
            self.config = yaml.load(configFile)
            configFile.close()
        except IOError:
            raise NTRTMasterError("Please provide a valid configuration file")

        self.trialDirectory = os.path.abspath(self.RESOURCE_DIRECTORY_NAME + self.config['PathInfo']['trialPath'])
        dictTools.tryMakeDir(self.trialDirectory)
        # dictTools.tryMakeDir(self.trialDirectory + '/' + self.GENERAL_DIRECTORY_NAME)

        # Logging not behaving as expected
        # Pretty much just a console print right now
        self.logFileName = self.trialDirectory + '/' + self.LOG_FILE_NAME
        logging.getLogger("NTRT Learning")
        logging.basicConfig(filename=self.logFileName, format='%(asctime)s:%(levelname)s:%(message)s', level=logging.DEBUG)

        # Hack for terrain compatibility
        # Backwards Compatibility

        terrains = []
        for terrain in self.config['TrialProperties']['terrains']:
            try:
                terrains.append(self.TERRAINS[terrain])
            except Exception:
                raise Exception("Could not find terrain of type " + terrain + " in terrainMap.")

        # Backwards compatibility
        self.config['TrialProperties']['terrains'] = terrains

        self.trialProperties = self.config['TrialProperties']

    memberTemplate = None
    def getMemberTemplate(self):
        if not self.memberTemplate:
            seedDirectory = self.config['PathInfo']['seedDirectory']
            templateFilePath = seedDirectory + os.listdir(seedDirectory)[0]
            print templateFilePath
            self.memberTemplate = dictTools.loadFile(templateFilePath)
        return self.memberTemplate

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

    def getComponentsKey(self):
        return NotImplementedError("ComponentsPath must be implemented in subclass.")

    def getMemberFileType(self):
        seedDirectory = self.config['PathInfo']['seedDirectory']
        templateFilePath = seedDirectory + os.listdir(seedDirectory)[0]
        filename, fileExtension = os.path.splitext(templateFilePath)
        return fileExtension

    # This should be moved to the member class
    # Assumes a fully structured member, with components
    # ALSO ASSIGNS MEMBER's filePath FIELD
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
        member.filePath = filePath
        dictTools.dumpFile(memberTemplate, filePath)
        return basename

    def generateComponentPopulation(self, componentConfig, componentPopulation):
        trialProperties = self.config["TrialProperties"]
        #print "genID in generateComponentPopulation: " + str(generationID)
        # templateComponent = self.generateTemplateComponent(componentConfig)
        #dictTools.printDict(componentConfig)
        #dictTools.pause("Check output.")
        newComponentPopulation = dispatchLearning(componentConfig=componentConfig,
                                                  scoreMethod=trialProperties["scoreMethod"],
                                                  fitnessFunction=trialProperties["fitnessFunction"],
                                                  # templateComponent=templateComponent,
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

    # Better to pass the generation object instead of doing this
    # If not used by commit, delete this
    def getPreviousComponentGeneration(self, component, previousGeneration):
        previousComponentGeneration = []
        if len(previousGeneration.getMembers()) > 0:
            for member in previousGeneration.getMembers():
                previousComponent = member.components[component]
                previousComponentGeneration.append(previousComponent)
        return previousComponentGeneration

    def getNewGenerationID(self, previousGeneration=None):
        if previousGeneration:
            newID = previousGeneration.ID + 1
        else:
            newID = -1
        return newID

    def beginTrial(self):

        generationCount = self.trialProperties['generationCount']

        jobList = []

        previousGeneration = self.importSeedGeneration()
        for genNum in range(generationCount):
            activeGeneration = self.generationGenerator(previousGeneration)
            # dictTools.pause("PAUSE IN LEARNINGJOBMASTER LINE 207")

            for member in activeGeneration.getMembers():

                # Expecting a json-compatible dictionary
                fileName = self.writeMemberToFile(member)
                # dictTools.pause("fileName: " + fileName)

                for terrain in self.trialProperties['terrains']:
                    # TODO check if these args need to be passed from the yamlconfig
                    args = {'filename' : fileName,
                            'resourcePrefix' : self.RESOURCE_DIRECTORY_NAME,
                            'path'     : self.config['PathInfo']['trialPath'],
                            'executable' : self.config['PathInfo']['executable'],
                            'length'   : self.trialProperties['trialLength'],
                            'terrain'  : terrain}
                    # Pass member by reference for updating scores
                    jobList.append(LearningJob(args, member))

            # Run the jobs
            # Only controller simulation is implemented right now
            if self.trialProperties['learningType'] == "controller":
                conSched = ConcurrentScheduler(jobList, self.numProcesses)
                completedJobs = conSched.processJobs()

            # for job in completedJobs:
            #     job.processJobOutput()
            #     scores = job.obj['scores']
            #     paramID = job.obj['paramID']
            #     activeGeneration.getMembers(paramID)

            previousGeneration = activeGeneration

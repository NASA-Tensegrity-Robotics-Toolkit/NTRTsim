import os
import json
import yaml
import logging
import random
from interfaces import NTRTJobMaster, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
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
    GENERAL_DIRECTORY_NAME = "OutputMembers/"

    # File Names
    LOG_FILE_NAME              = "output.log"
    LOGGING_NAME               = "NTRT Learning"
    SUMMARY_FILE_NAME          = "summary.txt"

    # Protected Words
    # These are words used in defining basic learning properties in YAML
    PROTECTED_TERMS = [
        # These two are currently used when extracting component libraries
        "PathInfo",
        "TrialProperties",
        # These are added as a precaution. Remove if not used after multiple commits.
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

    # Change to a YAML config file later
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
        dictTools.tryMakeDir(self.trialDirectory + '/' + self.GENERAL_DIRECTORY_NAME)

        # Logging not behaving as expected
        # Pretty much just a console print right now
        self.logFileName = self.trialDirectory + '/' + self.LOG_FILE_NAME
        logging.getLogger("NTRT Learning")
        logging.basicConfig(filename=self.logFileName, format='%(asctime)s:%(levelname)s:%(message)s', level=logging.DEBUG)

        # Hack for terrain compatibility
        # Backwards Compatibility

        terrainMap = {
            "flat" : [[0, 0, 0, 0]],
            "hill" : [[0, 0, 0, 0.0, 60000]]
        }

        terrains = []
        for terrain in self.config['TrialProperties']['terrains']:
            try:
                terrains.append(terrainMap[terrain])
            except Exception:
                raise Exception("Could not find terrain of type " + terrain + " in terrainMap.")

        # Backwards compatibility
        self.config['TrialProperties']['terrains'] = terrains

        self.trialProperties = self.config['TrialProperties']

    def getComponentsDictionary(self):
        componentsDictionary = {}
        for key in self.config:
            #if key == 'PathInfo' or key == 'TrialProperties':
            if key in self.PROTECTED_TERMS:
                pass
            else:
                componentsDictionary[key] = self.config[key]
        return componentsDictionary

    # This should be moved to the member class
    # Assumes a fully structured member, with components
    def writeMemberToFile(self, member):
        components = member.components
        #dictTools.printDict(components)
        #dictTools.pause()
        #print self.config['PathInfo']['fileName']
        # for key in components:
        #    print key
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

    # ID is id of previousGeneration + 1
    # if not previousGeneration then ID = 0
    def createNewGeneration(self, componentPopulations, generationID):
        newGeneration = Generation(generationID)
        # print "genID in createNewGeneration: " + str(generationID)
        # Not really using id here...
        for id in range(self.config['TrialProperties']['generationSize']):
            #print "newMember memID & genID: " + str(id) +" "+str(generationID)
            newMember = Member(memberID=id,generationID=generationID)
            #print "newMember memID & genID: " + str(newMember.components['memberID']) +" "+str(newMember.components['generationID'])
            #print "size of controller.components: " + str(len(newMember.components))
            #print "--"
            #dictTools.pause()
            # if "edgeVals" in newMember.components:
            #    dictTools.printDict(newMember.components['edgeVals'])
            # Dictionaries here might be pass by ref instead of pass by val
            for component, population in componentPopulations.iteritems():
                randomPopulation = random.choice(population)
                newMember.components[component] = randomPopulation
            #dictTools.printDict(newMember.components['edgeVals'])
            newGeneration.addMember(newMember)

        return newGeneration

    def importSeedGeneration(self):
        seedDirectory = self.config['PathInfo']['seedDirectory']
        logging.info(seedDirectory)
        id = 0
        previousGeneration = Generation(-1)
        if os.path.isdir(seedDirectory):
            for file in os.listdir(seedDirectory):
                absFilePath = os.path.abspath(seedDirectory) + '/' + file
                logging.info(absFilePath)
                seedFile = open(absFilePath, 'r')
                seedInput = json.load(seedFile)
                seedFile.close()
                newMember = Member(components=seedInput)
                previousGeneration.addMember(newMember)
                id += 1
        else:
            #raise Exception("Trying to import from a seed directory that is not a seed directory. Check the seedDirectory element in PathInfo.")
            print "Trying to import from a seed directory that is not a seed directory. Check the seedDirectory element in PathInfo."

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

    # This is a really hacky solution
    def beginTrialMaster(self, generationGeneratorFuction):

        generationCount = self.trialProperties['generationCount']
        populationSize = self.trialProperties['generationSize']

        jobList = []

        previousGeneration = self.importSeedGeneration()
        for genNum in range(generationCount):
            # We want to write all of the trials for post processing
            activeGeneration = generationGeneratorFuction(previousGeneration)

            for member in activeGeneration.getMembers():

                # Expecting a json-compatible dictionary
                fileName = self.writeMemberToFile(member)
                # dictTools.pause("fileName: " + fileName)

                for terrain in self.trialProperties['terrains']:
                    # All args to be passed to subprocess must be strings
                    # TODO check if these args need to be passed from the yamlconfig

                    args = {'filename' : fileName,
                            'resourcePrefix' : self.RESOURCE_DIRECTORY_NAME,
                            'path'     : self.config['PathInfo']['trialPath'],
                            'executable' : self.config['PathInfo']['executable'],
                            'length'   : self.trialProperties['trialLength'],
                            'terrain'  : terrain}
                    jobList.append(LearningJob(args))

            # Run the jobs
            conSched = ConcurrentScheduler(jobList, self.numProcesses)
            completedJobs = conSched.processJobs()

            for job in completedJobs:
                job.processJobOutput()

            previousGeneration = activeGeneration

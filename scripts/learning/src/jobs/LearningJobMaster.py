import os
import json
import yaml
import logging
from interfaces import NTRTJobMaster, NTRTMasterError
from concurrent_scheduler import ConcurrentScheduler
import collections
#TODO: This is hackety, fix it.
from evolution import evolution_job
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

        self.trialProperties = self.config['TrialProperties']
        self.trialDirectory = os.path.abspath(self.RESOURCE_DIRECTORY_NAME + self.config['PathInfo']['trialPath'])
        dictTools.tryMakeDir(self.trialDirectory)
        dictTools.tryMakeDir(self.trialDirectory + '/' + self.GENERAL_DIRECTORY_NAME)

        # Logging not behaving as expected
        # Pretty much just a console print right now
        self.logFileName = self.trialDirectory + '/' + self.LOG_FILE_NAME
        logging.getLogger("NTRT Learning")
        logging.basicConfig(filename=self.logFileName, format='%(asctime)s:%(levelname)s:%(message)s', level=logging.DEBUG)

        # Hack for terrain compatibility
        if self.config['TrialProperties']['terrains'] == "flat":
            self.terrains = [[[0, 0, 0, 0]]]

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


    def importSeedMembers(self):
        seedDirectory = self.config['PathInfo']['seedDirectory']
        logging.info(seedDirectory)
        seedMembers = []
        id = 0
        if os.path.isdir(seedDirectory):
            for file in os.listdir(seedDirectory):
                absFilePath = os.path.abspath(seedDirectory) + '/' + file
                logging.info(absFilePath)
                seedFile = open(absFilePath, 'r')
                seedInput = json.load(seedFile)
                seedFile.close()
                newMember = Member(components=seedInput)
                seedMembers.append(newMember)
                id += 1
        else:
            #raise Exception("Trying to import from a seed directory that is not a seed directory. Check the seedDirectory element in PathInfo.")
            print "Trying to import from a seed directory that is not a seed directory. Check the seedDirectory element in PathInfo."

        previousGeneration = Generation(-1)
        return seedMembers

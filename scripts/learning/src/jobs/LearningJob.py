import os
import subprocess
import sys
import json
import logging
from interfaces import NTRTMasterError, NTRTJob



class LearningJob(NTRTJob):

    def __init__(self, jobArgs, member):
        logging.info("Constructing job with args %r" % jobArgs)
        self.args = jobArgs
        self.member = member

        self._setup()

    # Unused
    # Must be implemented since LearningJob implements the NTRTJob interface
    def _setup(self):
        pass

    def startJob(self):
        logging.info("STARTING job with args %r" % self.args)
        self.pid = os.fork()

        if self.pid == 0:
            # Redirect the stdout output to dev null in the child.
            logPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename'] + '_log.txt'
            logFile = open(logPath, 'wb')

            # A set of jobs. Currently [0 0] is flat ground, [1 0] is a block field, [0 1] is hilly terrain, and [1 1] is both
            # This will expand in the future.
            terrainMatrix = self.args['terrain']
            # Update this if the subprocess call gets changed
            if len(terrainMatrix[0]) < 4:
                raise NTRTMasterError("Not enough terrain args!")

            # Run through a set of binary job options. Currently handles terrain switches
            for run in terrainMatrix:
                if (len(run)) >= 5:
                    trialLength = run[4]
                else:
                    trialLength = self.args['length']
                #TODO improve error handling here
                """
                raw_input("Preparing process with:")
                for arg in [self.args['executable'], "-l", self.args['filename'], "-P", self.args['path'], "-s", str(trialLength), "-b", str(run[0]), "-H", str(run[1]), "-a", str(run[2]), "-B", str(run[3])]:
                    print arg
                raw_input("About to execute process.")
                """
                subprocess.check_call([self.args['executable'], "-l", self.args['filename'], "-P", self.args['path'], "-s", str(trialLength), "-b", str(run[0]), "-H", str(run[1]), "-a", str(run[2]), "-B", str(run[3])], stdout=logFile)
                # raw_input("Process executed.")
            sys.exit()

    # Object/dictionary was written to in C++, so we get the updated version here
    # Why not just perform the component/NN updates in C++?
    def processJobOutput(self):
        scoresPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename']

        try:
            fin = open(scoresPath, 'r')
            self.obj = json.load(fin)
            fin.close()
        except IOError:
            self.obj = {}


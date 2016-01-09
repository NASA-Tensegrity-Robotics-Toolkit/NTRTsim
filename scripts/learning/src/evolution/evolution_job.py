import os
import subprocess
import sys
import json
import logging
from interfaces import NTRTMasterError, NTRTJob



class EvolutionJob(NTRTJob):

    def __init__(self, jobArgs):
        """
        Override this in your subclass. Be sure that at the end of your method your init method
        you make a call to self._setup(). I'll clean this up later so that we're properly doing a super
        call (rather than invoking setup in the child), no need for you to handle that now.

        You can put args into this however you want, just depends on what convention you want to use. I'd personally
        use a dictionary. If you use a dictionary, just use the jobArgs keyword from this function's signature.
        """
        logging.info("Constructing job with args %r" % jobArgs)
        self.args = jobArgs

        self._setup()

    def _setup(self):
        """
        This is where you'll handle setup related to this *single* learning trial. Each instance of NTRT
        we run will have its own NTRTJob instance.
        """

    def startJob(self):
        """
        Override this to start the NTRT instance and pass it the relevant parameters.. This is called
        by NTRTJobMaster when it wants to start this NTRT process.
        """

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
                subprocess.check_call([self.args['executable'], "-l", self.args['filename'], "-P", self.args['path'], "-s", str(trialLength), "-b", str(run[0]), "-H", str(run[1]), "-a", str(run[2]), "-B", str(run[3])], stdout=logFile)
            sys.exit()

    def processJobOutput(self):
        scoresPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename']

        try:
            fin = open(scoresPath, 'r')
            self.obj = json.load(fin)
            fin.close()
        except IOError:
            self.obj = {}


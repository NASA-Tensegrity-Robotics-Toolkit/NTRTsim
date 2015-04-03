#!/usr/bin/python

# Copyright (c) 2012, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All rights reserved.
#
# The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
# under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0.
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

""" Converts .nnw files to a JSON file """

# Purpose: Queue up learning runs, pass them parameters via JSON
# Author:  Brian Mirletz and Perry Bhandal
# Date:    March 2015
# Notes:   In progress as of this commit (3/27/15)

import sys
import os
import subprocess
import json
import random
import logging
from concurrent_scheduler import ConcurrentScheduler

###
# Interfaces.
##

class NTRTJobMaster:
    """
    One NTRTJobMaster will exist for the entire learning run. It's responsible for managing our
    NTRTJob objects.
    """

    def __init__(self, configFile, numProcesses):
        """
        Don't override init. You should do all of your setup in the _setup method instead.
        """
        logging.info("Instatiated NTRTJobMaster. Config file is %s, using %d processes." % (configFile, numProcesses))
        self.configFileName = configFile
        self.numProcesses = numProcesses

        self._setup()

    def _setup(self):
        """
        Override this method and implement any global setup necessary. This includes tasks
        like creating your input and output directories.
        """
        raise NotImplementedError("")

    def beginTrial(self):
        """
        Override this. It should just contain a loop where you keep constructing NTRTJobs, then calling
        runJob on it (which will block you until the NTRT instance returns), parsing the result from the job, then
        deciding if you should run another trial or if you want to terminate.
        """
        raise NotImplementedError("")

class NTRTJob:

    def __init__(self, jobArgs):
        """
        Override this in your subclass. Be sure that at the end of your method your init method
        you make a call to self._setup(). I'll clean this up later so that we're properly doing a super
        call (rather than invoking setup in the child), no need for you to handle that now.

        You can put args into this however you want, just depends on what convention you want to use. I'd personally
        use a dictionary. If you use a dictionary, just use the jobArgs keyword from this function's signature.
        """
        raise NotImplementedError("")

    def _setup(self):
        """
        This is where you'll handle setup related to this *single* learning trial. Each instance of NTRT
        we run will have its own NTRTJob instance.
        """
        raise NotImplementedError("")

    def startJob(self):
        """
        Override this to start the NTRT instance and pass it the relevant parameters.. This is called
        by NTRTJobMaster when it wants to start this NTRT process.
        """
        raise NotImplementedError("")

    def processJobOutput(self):
        """
        This method will be called once this job is complete (we define 'complete' as the process forked
        in startJob ends).
        """
        raise NotImplementedError("")

    def cleanup(self):
        """
        You can override this if you want and handle cleaning up any output files from this job. Not really necessary
        though, I can take care of that myself later.
        """
        pass

class NTRTMasterError(Exception):
    """
    Base class for exceptions in this module
    """
    pass

###
# Your implementations.
###

class BrianJobMaster(NTRTJobMaster):
    def _setup(self):
        """
        Override this method and implement any global setup necessary. This includes tasks
        like creating your input and output directories.
        """

        # If this fails, the program should fail. Input file is required
        # for useful output
        try:
            fin = open(self.configFileName, 'r')
            self.jConf = json.load(fin)
            fin.close()
        except IOError:
            raise NTRTMasterError("Please provide a valid configuration file")

        self.path = self.jConf['resourcePath'] + self.jConf['lowerPath']

        try:
            os.makedirs(self.path)
        except OSError:
            if not os.path.isdir(self.path):
                raise NTRTMasterError("Directed the folder path to an invalid address")

        # Consider seeding random, using default (system time) now

    def __getNewParams(self, paramName):
        """
        Generate a new set of paramters based on learning method and config file
        Returns a dictionary paramName : values
        This version is Monte Carlo, other versions may just pick up a pre-generated file
        """
        params = self.jConf["learningParams"][paramName]
        if params['numberOfStates'] == 0 :

            newParams = []

            for i in range(0, params['numberOfInstances']) :
                subParams = []
                for j in range(0, params['numberOfOutputs']) :
                    # Assume scaling happens elsewhere
                    subParams.append(random.random())
                newParams.append(subParams)
        else :
            # Temp code for pre-specified neural network, future editions will generate networks
            newParams = { 'numActions' : params['numberOfOutputs'],
                         'numStates' : params['numberOfStates'],
                         'neuralFilename' : "logs/bestParameters-6_fb-0.nnw"}

        return newParams

    def getNewFile(self, jobNum):
        """
        Handle the generation of a new JSON file with new parameters. Will vary based on the
        learning method used and the config file
        """

        obj = {}

        obj["nodeVals"] = self.__getNewParams("nodeVals")
        obj["edgeVals"] = self.__getNewParams("edgeVals")
        obj["feedbackVals"] = self.__getNewParams("feedbackVals")

        outFile = self.path + self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

        fout = open(outFile, 'w')

        json.dump(obj, fout, indent=4)

        return self.jConf['filePrefix'] + "_" + str(jobNum) + self.jConf['fileSuffix']

    def beginTrial(self):
        """
        Override this. It should just contain a loop where you keep constructing NTRTJobs, then calling
        runJob on it (which will block you until the NTRT instance returns), parsing the result from the job, then
        deciding if you should run another trial or if you want to terminate.
        """

        numTrials = self.jConf['learningParams']['numTrials']

        results = {}
        jobList = []

        for i in range(1, numTrials) :

            # MonteCarlo solution. This function could be overridden with something that
            # provides a filename for a pre-existing file
            fileName = self.getNewFile(i)

            # All args to be passed to subprocess must be strings
            args = {'filename' : fileName,
                    'resourcePrefix' : self.jConf['resourcePath'],
                    'path'     : self.jConf['lowerPath'],
                    'executable' : self.jConf['executable'],
                    'length'   : self.jConf['learningParams']['trialLength']}
            jobList.append(BrianJob(args))

        conSched = ConcurrentScheduler(jobList, self.numProcesses)
        completedJobs = conSched.processJobs()

        for job in completedJobs:
            job.processJobOutput()
            fileName = job.args['filename']
            results[fileName] = job.obj[0]['distance']

        print "Jobs are complete, results are: %r" % results
        #job.startJob()
        #scores = job.runJob()
        #results[fileName] = scores[0]['distance']

        #TODO, something that exports results and picks the best trial based on results


class BrianJob(NTRTJob):

    def __init__(self, jobArgs):
        """
        Override this in your subclass. Be sure that at the end of your method your init method
        you make a call to self._setup(). I'll clean this up later so that we're properly doing a super
        call (rather than invoking setup in the child), no need for you to handle that now.

        You can put args into this however you want, just depends on what convention you want to use. I'd personally
        use a dictionary. If you use a dictionary, just use the jobArgs keyword from this function's signature.
        """
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

        self.pid = os.fork()

        if self.pid == 0:
            # Redirect the stdout output to dev null in the child.
            sys.stdout = os.devnull
            subprocess.call([self.args['executable'], "-l", self.args['filename'], "-s", str(self.args['length'])])
            sys.exit(0)

    def processJobOutput(self):
        scoresPath = self.args['resourcePrefix'] + self.args['path'] + self.args['filename']

        try:
            fin = open(scoresPath, 'r')
            self.obj = json.load(fin)
            fin.close()
        except IOError:
            self.obj = {}

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    configFile = sys.argv[1]
    numProcesses = int(sys.argv[2])
    jobMaster = BrianJobMaster(configFile, numProcesses)
    jobMaster.beginTrial()

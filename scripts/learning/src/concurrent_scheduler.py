import logging
import time
import psutil
import os
import json
from operator import itemgetter

class ConcurrentScheduler:

    # Number of seconds to wait after spawning processes before
    # querying PID list.
    __CHECK_DELAY = 0.1

    def __init__(self, toProcess, numProcesses):
        self.numProcesses = numProcesses
        self.jobsUnprocessed = toProcess
        self.jobsProcessing = []
        self.jobsComplete = []
        self.jobsTop2N = [] # To store only the top 2n jobs completed, in the case of Monte Carlo
        self.jobsRemaining = [] # To store return value, as there are now two different things being returned between MC and evolution
        self.top2n = [] # To store info on the best 2n controllers
        self.sortedTop2n = [] # To store the sorted list of the best 2n controllers
        self.mcScores = [] # To store distance and controller number for all trials
        logging.info("Concurrent Scheduler instantiated. Contains %d jobs. Number of concurrent processes: %d." % (len(self.jobsUnprocessed), self.numProcesses))

    def processJobs(self, mc, best2n):
        logging.info("Concurrent scheduler beginning jobs.")
        
        top2nDump = open('top2nDump.txt', 'w') # Clear top2n file
        top2nDump.close()

        mcScoresDump = open('mcScoresDump.txt', 'w') # Clear mcScores file
        mcScoresDump.close()

        self.__jobLoop(mc, best2n)
        
        # Since we won't have all the completed job files anymore at this point, if doing Monte Carlo
        if(mc):
            self.jobsRemaining = self.jobsTop2N
        else:
            self.jobsRemaining = self.jobsComplete
        return self.jobsRemaining

    def __jobLoop(self, mc, best2n):
        while True:

            if len(self.jobsUnprocessed) == 0 and len(self.jobsProcessing) == 0:
                logging.info("All jobs processed. Breaking out of job loop.")
                if(mc):

                    # Sort the top 2n scores, from longest distance traveled to shortest:
                    self.sortedTop2n = sorted(self.top2n, key=itemgetter('distance'), reverse=True)

                    # Dump the top 2n scores into a .txt file when the simulation is complete
                    # May need to move this to check_processes() (under 'if minDist < distance) 
                    # if going to run infinitely many trials until 5 days done, and simulation may abort early.
                    # But for now, it's done here
                    top2nDump = open('top2nDump.txt', 'a')
                    top2nDump.write(json.dumps(self.sortedTop2n))
                    top2nDump.write('\n')
                    top2nDump.close()

                    # Dump all MC scores into a .txt file when simulation is complete. 
                    # Same deal as for top2n, this may be moved in the future.
                    mcScoresDump = open('mcScoresDump.txt', 'a')
                    mcScoresDump.write(json.dumps(self.mcScores))
                    mcScoresDump.write('\n')
                    mcScoresDump.close()
                return

            while len(self.jobsProcessing) < self.numProcesses and len(self.jobsUnprocessed) > 0:
                logging.info("Spawning an additional job. Number of active jobs before is %d." % len(self.jobsProcessing))
                toRun = self.jobsUnprocessed.pop()
                self.jobsProcessing.append(toRun)
                toRun.startJob()

            time.sleep(self.__CHECK_DELAY)

            self.__checkProcesses(mc, best2n)

    def __checkProcesses(self, mc, best2n):
        completed = []
        # making sure the parameter was passed correctly

        for activeProc in self.jobsProcessing:
            proc = psutil.Process(activeProc.pid)
            if proc.status() == psutil.STATUS_ZOMBIE:
                logging.info("Process with ID %d is now a zombie process. Mark it as complete." % activeProc.pid)
                completed.append(activeProc)
                # Releasing the processID from each zombie process
                childPid, status = os.waitpid(activeProc.pid, 0)
                logging.info("Zombie process ID %d, with exit status %d, has been removed from the process table." % (childPid, status))


        for completeProc in completed:
            self.jobsProcessing.remove(completeProc)
            self.jobsComplete.append(completeProc)
            if(mc):
                # Get the scores from the completed trial
                completeProc.processJobOutput()
                jobVals = completeProc.obj
                scores = jobVals['scores']
                # Extract the trial number out of 'filename'. 
                # 'fileNumList' will only be a list of one element, so this seems a roundabout way but it works. 
                # Others are welcome to find a better way
                splitFileNameList = completeProc.args['filename'].split('.')
                fileNameList = [int(s) for s in splitFileNameList[0].split('_') if s.isdigit()]
                trialNum = fileNameList.pop()
                # print trialNum
                trial = {}
                distance = 0

                # Extract the distance from the scores for the completed trial
                for i in scores:
                    distance = i['distance']
                    # print distance
                # Then put this in the big mcScores array, along with trial number 
                trial = {'trialnum': trialNum, 'distance': distance}
                self.mcScores.append(trial)

                # The following seems redundant, but appending this to top2n instead of 'trial' 
                # prevents the first 2n entries of both the mcScores and top2n list objects from 
                # pointing to the same memory location. Otherwise, the first 2n of mcScores and top2n will always be the same. 
                bestTrial = {'trialnum': trialNum, 'distance': distance}

                # See if the distance in completeProc is in 2n, 
                # if so, add it. If not, delete it!
                if len(self.top2n[:]) < best2n:  
                    self.top2n.append(bestTrial)
                    self.jobsTop2N.append(completeProc)
                else:
                    minDist = float('inf')
                    minDistIdx = -1
                    minTrialNum = -1
                    for i in range(0, len(self.top2n[:])):
                        if self.top2n[i]['distance'] < minDist:
                            minDist = self.top2n[i]['distance']
                            minTrialNum = self.top2n[i]['trialnum'] # This is the number of the MC file that needs to be deleted.
                            minDistIdx = i
                    # if distance is greater than the minimum, 
                    # copy the new distance and trialNum to that index
                    if minDist < distance:
                        self.top2n[minDistIdx]['distance'] = bestTrial['distance']
                        self.top2n[minDistIdx]['trialnum'] = bestTrial['trialnum']

                        # Add the completed job to the top 2n jobs completed so far
                        self.jobsTop2N.append(completeProc)

                        # In the future, may need to dump the top2n and mcScores into files here, 
                        # But for now this is done in __jobLoop() after all trials are complete

                        # Reconstruct the filename of the old job to remove from jobs completed so far
                        discardFileName = 'monteOut_' + str(minTrialNum) + '.json'
                        # Then find and remove it
                        removeIdx = -1
                        for i in range (0, len(self.jobsTop2N[:])):
                            jobToDiscard = self.jobsTop2N[i]
                            if jobToDiscard.args['filename'] == discardFileName:
                                removeIdx = i
                        if removeIdx >= 0:
                            self.jobsTop2N.pop(removeIdx)
                        # Find some way to printf self.jobsTop2N, in case of errors, to debug!


                        # now delete the old file that was removed from top2n
                        # Reconstruct the file paths corresponding to the trialnum that was just removed from top2n
                        discardFilePath = completeProc.args['resourcePrefix'] + completeProc.args['path'] + discardFileName
                        discardLogPath = completeProc.args['resourcePrefix'] + completeProc.args['path'] + 'monteOut_' + str(minTrialNum) + '.json_log.txt' 
                        
                    else:
                       discardFilePath = completeProc.args['resourcePrefix'] + completeProc.args['path'] + completeProc.args['filename']
                       discardLogPath = discardFilePath + '_log.txt'
                        
                    # open file at discardFilePath first, to retrieve the name of the corresponding .nnw file!
                    try:
                        fin = open(discardFilePath, 'r')
                        self.oldJobVals = json.load(fin)
                        fin.close()
                    except IOError:
                        self.oldJobVals = {}

                    nnwFilePath = completeProc.args['resourcePrefix'] + completeProc.args['path'] + self.oldJobVals['feedbackVals']['params']['neuralFilename']

                    # Delete this nnw file first, then the monteOut_ files
                    try:
                        os.remove(nnwFilePath)
                    except OSError:
                        raise NTRTMasterError("Please provide a valid .nnw file path")
                    try:
                        os.remove(discardFilePath)
                    except OSError:
                        raise NTRTMasterError("Please provide a valid .json file path")
                    try:
                        os.remove(discardLogPath)
                    except OSError:
                        raise NTRTMasterError("Please provide a valid .txt file path")

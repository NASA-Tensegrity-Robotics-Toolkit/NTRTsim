import logging
import time
import psutil
import os

class ConcurrentScheduler:

    # Number of seconds to wait after spawning processes before
    # querying PID list.
    __CHECK_DELAY = 0.1

    def __init__(self, toProcess, numProcesses):
        self.numProcesses = numProcesses
        self.jobsUnprocessed = toProcess
        self.jobsProcessing = []
        self.jobsComplete = []
        logging.info("Concurrent Scheduler instantiated. Contains %d jobs. Number of concurrent processes: %d." % (len(self.jobsUnprocessed), self.numProcesses))

    def processJobs(self):
        logging.info("Concurrent scheduler beginning jobs.")
        self.__jobLoop()
        return self.jobsComplete

    def __jobLoop(self):
        while True:

            if len(self.jobsUnprocessed) == 0 and len(self.jobsProcessing) == 0:
                logging.info("All jobs processed. Breaking out of job loop.")
                return

            while len(self.jobsProcessing) < self.numProcesses and len(self.jobsUnprocessed) > 0:
                logging.info("Spawning an additional job. Number of active jobs before is %d." % len(self.jobsProcessing))
                toRun = self.jobsUnprocessed.pop()
                self.jobsProcessing.append(toRun)
                toRun.startJob()

            time.sleep(self.__CHECK_DELAY)

            self.__checkProcesses()

    def __checkProcesses(self):
        completed = []

        for activeProc in self.jobsProcessing:
            proc = psutil.Process(activeProc.pid)
            if proc.status() == psutil.STATUS_ZOMBIE:
                logging.info("Process with ID %d is now a zombie process. Mark it as complete." % activeProc.pid)
                completed.append(activeProc)
                # Releasing the processID from each zombie process
                childPid, status = os.waitpid(activeProc.pid, 0)
                logging.info("Zombie process ID %d, with exit status %d, has been removed from the process table and can be reused." % (childPid, status))

        for completeProc in completed:
            self.jobsProcessing.remove(completeProc)
            self.jobsComplete.append(completeProc)

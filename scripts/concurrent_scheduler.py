class ConcurrentScheduler:

    def self(self, toProcess, numProcesses):
        self.numProcesses = numProcesses
        self.jobsUnprocessed = toProcess
        self.jobsProcessing = []
        self.jobsComplete = []

    def processJobs(self):
        print "In process jobs."

        return self.jobsComplete

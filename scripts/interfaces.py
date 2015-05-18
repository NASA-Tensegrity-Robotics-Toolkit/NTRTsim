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


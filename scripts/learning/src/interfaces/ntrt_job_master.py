import logging

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


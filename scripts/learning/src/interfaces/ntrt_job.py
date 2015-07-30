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


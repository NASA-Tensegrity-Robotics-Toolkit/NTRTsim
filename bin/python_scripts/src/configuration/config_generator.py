class ConfigGenerator:

    def __init__(self, generatedDir, defaultDir):
        """
        Creates a ConfigGenerator object.

        Parameters

        generatedDir: Relative path (relative to base dir) where generated conf files are written.
        defaultDir: Relative path (relative to base dir) to where default configuration files are located.
        """
        self.generatedDir = generatedDir
        self.defaultDir = defaultDir

    def getMissing(self):
        """
        Returns a list containing the names of all missing conf files.
        """


    def generateMissing(self):
        """
        Generates all missing conf files.
        """
        pass

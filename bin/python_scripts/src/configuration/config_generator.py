from ..utilities.directory import Directory
import shutil

class ConfigGenerator:

    def __init__(self, generatedDir, defaultDir):
        """
        Creates a ConfigGenerator object.

        Parameters

        generatedDir: Absolute path where generated conf files are written.
        defaultDir: Absolute path where conf templates can be found.
        """
        self.generatedDir = generatedDir
        self.defaultDir = defaultDir

    def getMissing(self):
        """
        Returns a list containing the names of all missing conf files.
        """
        generatedDirectory = Directory(self.generatedDir).getDirList()
        defaultDirectory = Directory(self.defaultDir).getDirList()

        toGenerate = []
        for defaultFile in defaultDirectory:
            generatedName = defaultFile.rsplit(".", 1)
            if generatedName[0] not in generatedDirectory:
                toGenerate.append(defaultFile)

        return toGenerate

    def generateMissing(self):
        """
        Generates all missing conf files.
        """
        # Get the list of missing files.
        missingList = self.getMissing()

        # Loop over the missing list. For each one copy it.
        for missing in missingList:
            srcPath = "%s/%s" % (self.defaultDir, missing)
            destPath = "%s/%s" % (self.generatedDir, missing.rsplit(".", 1)[0])

            shutil.copy(srcPath, destPath)

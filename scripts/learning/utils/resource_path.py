import os

class ResourcePath:

    def __init__(self, resourcePath):
        #TODO: Need to follow proper docstring conventions. Need to do this across the board, especially for constructors -- basically anywhere we have parameters. Double check whether Vim has an auto generator based on the function signature to generate the skeleton.
        """
        Keyword arguments:

        resourcePath - The path relative to NTRTsim's root to this resource path. This path should contain a trailing slash.
        """
        if not self.__isIndexSlash(resourcePath, -1):
            raise ResourcePathError("Received resource path %s, but this does not contain a trailing slash as expected.", resourcePath)
        self.resourcePath = resourcePath

    def createPath(self):
        """
        Creates this resource path if it doesn't exist.

        Throws ResourcePathError if any error occurs during path creation.
        """

        try:
            os.makedirs(self.resourcePath)
        except OSError, e:
            raise ResourcePathError("Resource path creation failed, hit an OSError. Error message is %s" % e)

    def getResourcePath(self, filePath):
        """
        Returns the path to filePath within this specific resource path. This path
        is relative to the simulator's root directory.

        Keyword arguments:

        filePath - The path relative to the resource path for this file. Note that this file path should *not* contain a slash at the beginning.
        """
        if self.__isIndexSlash(filePath, 0):
            raise ResourcePathError("Received file path %s includes an initial slash.", filePath)

        return "%s%s" % (self.resourcePath, filePath)

    def __isIndexSlash(self, toCheck, index):
        if toCheck[index] == '/':
            return True
        else:
            return False

class ResourcePathError(Exception):
    pass

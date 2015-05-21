import json

class ConfigLoader:

    def __init__(self, configFileName):
        """
        Constructs a ConfigLoader object, and attempts to load the provided file
        upon instantiation.
        """
        self.configFileName = configFileName
        self.__loadFile()

    def toDict(self):
        """
        Returns a dictionary containing the JSON file's contents.
        """
        return json.load(self.loadedConf)

    def __loadFile(self):
        """
        Attempts to load the provided path.

        Throws ConfigLoaderError if file opening fails for any reason.
        """
        try:
            self.loadedConf = open(self.configFileName, 'r')
        except IOError, e:
            raise ConfigLoaderError("Hit IOError while loading config file %s. Error message is %s" % (self.configFileName, e))

class ConfigLoaderError(Exception):
    pass

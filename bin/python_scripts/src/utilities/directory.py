import os

class Directory:

    def __init__(self, absPath):
        """
        Creates a Directory object.

        Parameters

        absPath: An absolute path to the directory this object operates over.
        """
        self.absPath = absPath

    def getDirList(self, extWhiteList=[]):
        """
        Returns a list containing all files in this directory.

        Parameters

        extWhiteList: Optional. If specified, only files which end with a path in extWhiteList will be returned.
        """
        rawList = [ f for f in os.listdir(self.absPath) if os.path.isfile(os.path.join(self.absPath,f)) ]

        # Remove blacklisted extensions (if needed)
        if len(extWhiteList) > 0:
            whiteListed = []

            for entry in rawList:
                for whiteFilter in extWhiteList:
                    if entry.endswith(whiteFilter):
                        whiteListed.append(entry)

            return whiteListed
        else:
            return rawList

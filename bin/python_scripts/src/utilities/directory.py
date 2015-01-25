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
        rawList = os.listdir(self.absPath)
        whiteListed = []
        removedDirs = []

        # Remove blacklisted extensions (if needed)
        if len(extWhiteList) > 0:
            for entry in rawList:
                for whiteFilter in extWhiteList:
                    if entry.endswith(whiteFilter):
                        whiteListed.append(entry)

        # Remove folders.
        for entry in whiteListed:
            if os.path.isfile("%s/%s" % (self.absPath, entry)):
                print "%s is a file." % entry
                removedDirs.append(entry)
            else:
                print "%s is a folder." % entry

        return removedDirs

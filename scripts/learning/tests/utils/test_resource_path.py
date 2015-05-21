import unittest
from flexmock import *
from utils import ResourcePath, ResourcePathError

class TestResourcePath(unittest.TestCase):

    def testRootPathNoTrailingSlashThrowsError(self):
        with self.assertRaises(ResourcePathError):
            ResourcePath("test/meow")

    def testRootPathHasTrailingSlashNoError(self):
        self.__getResPathObject()

    def testGetPathHasForwardSlashThrowsError(self):
        resPath = self.__getResPathObject()

        with self.assertRaises(ResourcePathError):
            resPath.getResourcePath("/relativePath/someOtherFile.tar.gz")

    def testGetPathNoForwardSlash(self):
        resPath = self.__getResPathObject()
        resPath.getResourcePath("relativePath/someOtherFile.tar.gz")

    def __getResPathObject(self):
        return ResourcePath("test/meow/")

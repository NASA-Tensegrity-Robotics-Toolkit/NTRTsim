import unittest
from flexmock import *
from utils import ResourcePath, ResourcePathError

class TestResourcePath(unittest.TestCase):

    def testRootPathNoTrailingSlashThrowsError(self):

        with self.assertRaises(ResourcePathError):
            ResourcePath("test/meow")

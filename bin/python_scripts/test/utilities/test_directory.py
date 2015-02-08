import unittest
from flexmock import *
from src.utilities.directory import *

class TestDirectory(unittest.TestCase):

    DEFAULT_LIST = ["bullet.ini.default", "folder", "folder.ini" "test.ini", "ini.bullet", "boost.ini.default", "test.txt"]

    DIR_ABS_PATH = "/home/perry/work"

    def setUp(self):
        flexmock(os).should_receive('listdir').and_return(self.DEFAULT_LIST)
        flexmock(os.path).should_receive('isfile').and_return(True)
        self.dirUnderTest = Directory(self.DIR_ABS_PATH)

    def testGetDirNoFilter(self):
        self.assertEqual(self.dirUnderTest.getDirList(), self.DEFAULT_LIST)

    def testGetDirOneFilter(self):
        filteredList = ["bullet.ini.default", "boost.ini.default"]
        self.assertEqual(self.dirUnderTest.getDirList([".ini.default"]), filteredList)

    def testGetDirTwoFilter(self):
        filteredList = ["bullet.ini.default", "ini.bullet", "boost.ini.default"]
        self.assertEqual(self.dirUnderTest.getDirList([".ini.default", ".bullet"]), filteredList)

    #TODO: Add a test here for folder removal.

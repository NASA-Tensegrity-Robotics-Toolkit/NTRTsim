import unittest
from flexmock import *
from src.utilities.directory import *

class TestDirectory(unittest.TestCase):

    DEFAULT_LIST_NO_FOLDERS = ["bullet.ini.default", "test.ini", "ini.bullet", "boost.ini.default", "test.txt"]

    DEFAULT_LIST_HAS_FOLDERS = ["bullet.ini.default", "default", "folder.ini.default", "test.ini", "ini.bullet", "boost.ini.default", "test.txt"]

    DIR_ABS_PATH = "/home/perry/work"

    def setUp(self):
        flexmock(os).should_receive('listdir').and_return(self.DEFAULT_LIST_NO_FOLDERS)
        self.dirUnderTest = Directory(self.DIR_ABS_PATH)

    def __setListDirReturn(self, toReturn):
        flexmock(os).should_receive('listdir').and_return(toReturn)

    def testGetDirNoFilter(self):
        self.__setListDirReturn(self.DEFAULT_LIST_NO_FOLDERS)
        self.assertEqual(self.dirUnderTest.getDirList(), self.DEFAULT_LIST_NO_FOLDERS)

    def testGetDirOneFilter(self):
        self.__setListDirReturn(self.DEFAULT_LIST_NO_FOLDERS)
        filteredList = ["bullet.ini.default", "boost.ini.default"]
        self.assertEqual(self.dirUnderTest.getDirList([".ini.default"]), filteredList)

    def testGetDirTwoFilter(self):
        self.__setListDirReturn(self.DEFAULT_LIST_NO_FOLDERS)
        filteredList = ["bullet.ini.default", "ini.bullet", "boost.ini.default"]
        self.assertEqual(self.dirUnderTest.getDirList([".ini.default", ".bullet"]), filteredList)

    def testIgnoreFoldersNoFilter(self):
        # Create the expected array.
        expectedDir = ["bullet.ini.default", "test.ini", "ini.bullet", "boost.ini.default", "test.txt"]

        flexmock(os.path).should_receive('isfile').with_args("%s/default" % self.DIR_ABS_PATH).and_return(False)
        flexmock(os.path).should_receive('isfile').with_args("%s/folder.ini.default" % self.DIR_ABS_PATH).and_return(False)

        for notDir in expectedDir:
            flexmock(os.path).should_receive('isfile').with_args("%s/%s" % (self.DIR_ABS_PATH, notDir)).and_return(True)

        # Set return type to return default list with folders.
        self.__setListDirReturn(self.DEFAULT_LIST_HAS_FOLDERS)

        # Check if the dir list is equal to what we expected.
        self.assertEqual(self.dirUnderTest.getDirList(), expectedDir)

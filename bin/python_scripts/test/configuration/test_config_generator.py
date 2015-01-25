import unittest
from flexmock import *
from src.configuration.config_generator import *

class TestConfigGenerator(unittest.TestCase):

    DEFAULT_LIST = ["bullet.ini.default", "gmocktest.ini.default", "general.ini.default"]

    CONF_GENERATED_DIR = "/home/perr/work/git/NTRTsim/conf"
    CONF_DEFAULT_DIR= "/home/perr/work/git/NTRTsim/conf"

    def setUp(self):
        self.configGenerator = ConfigGenerator(self.CONF_GENERATED_DIR, self.CONF_DEFAULT_DIR)

    def __setDirListReturn(self, generatedList):
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)

    def testNoMissing(self):
        generatedList = ["bullet.ini", "gmocktest.ini", "general.ini"]
        self.__setDirListReturn(generatedList)
        self.assertEqual(self.configGenerator.getMissing(), [])

    def testTwoMissing(self):
        generatedList = ["gmocktest.ini"]
        self.__setDirListReturn(generatedList)
        self.assertEqual(self.configGenerator.getMissing(), ["bullet.ini.default" ,"general.ini.default"])

    def testGenerateNoConf(self):
        generatedList = ["bullet.ini", "gmocktest.ini", "general.ini"]
        self.__setDirListReturn(generatedList)
        flexmock(shutil).should_receive('copy').times(0)
        self.configGenerator.generateMissing()

    def testGenerateTwoConf(self):
        generatedList = ["general.ini"]
        self.__setDirListReturn(generatedList)
        flexmock(shutil).should_receive('copy').times(2)
        self.configGenerator.generateMissing()


import unittest
from flexmock import *
from src.configuration.config_generator import *

class TestConfigGenerator(unittest.TestCase):

    DEFAULT_LIST = ["bullet.ini.default", "gmocktest.ini.default", "general.ini.default"]

    CONF_GENERATED_DIR = "conf"
    CONF_DEFAULT_DIR = "conf/default"

    def setUp(self):
        self.configGenerator = ConfigGenerator(self.CONF_GENERATED_DIR, self.CONF_DEFAULT_DIR)

    def testNoMissing(self):
        generatedList = ["bullet.ini", "gmocktest.ini", "general.ini"]
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)
        self.assertEqual(self.configGenerator.getMissing(), [])

    def testTwoMissing(self):
        generatedList = ["gmocktest.ini"]
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)
        self.assertEqual(self.configGenerator.getMissing(), ["bullet.ini.default" ,"general.ini.default"])

    def testGenerateNoConf(self):
        generatedList = ["bullet.ini", "gmocktest.ini", "general.ini"]
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)
        flexmock(shutil).should_receive('copy').times(0)
        self.configGenerator.generateMissing()

    def testGenerateTwoConf(self):
        generatedList = ["general.ini"]
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)
        flexmock(shutil).should_receive('copy').times(2)
        self.configGenerator.generateMissing()


import unittest
from flexmock import *
from src.configuration.config_generator import *

class TestConfigGenerator(unittest.TestCase):

    DEFAULT_LIST = ["bullet.ini.default", "gmocktest.ini.default", "general.ini.default"]

    def testGenerateNoConf(self):
        configGenerator = ConfigGenerator("conf", "conf/default")
        generatedList = ["bullet.ini", "gmocktest.ini", "general.ini"]
        flexmock(Directory).should_receive('getDirList').and_return(generatedList).and_return(self.DEFAULT_LIST)
        self.assertEqual(configGenerator.getMissing(), [])

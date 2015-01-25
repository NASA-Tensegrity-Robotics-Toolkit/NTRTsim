import unittest
from flexmock import *
from src.configuration.config_generator import *

class TestConfigGenerator(unittest.TestCase):

    def testGenerateNoConf(self):
        configGenerator = ConfigGenerator("conf", "conf/default")
        self.assertEqual(configGenerator.getMissing(), [])

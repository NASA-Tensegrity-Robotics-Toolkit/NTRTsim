# Copyright (c) 2012, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All rights reserved.
#
# The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
# under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0.
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language
# governing permissions and limitations under the License.


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


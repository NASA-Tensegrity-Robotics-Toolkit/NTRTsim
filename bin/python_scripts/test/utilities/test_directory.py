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

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
from src.download.downloader import *

# Helpers
#TODO: Make it so the entire mock helpers dir is automatically imported for every test.
from test.test_helpers.mock_helpers import MockHelpers

class TestDownloader(unittest.TestCase):

    URL = "http://www.perryb.ca/bigfile.tar.gz"
    LOCAL_SAVE_PATH = "/home/perry/work/git/NTRTsim/env/downloads"
    LOCAL_SAVE_NAME = "myFile.tar.gz"

    def setUp(self):
        self.downloader = Downloader(self.URL, self.LOCAL_SAVE_PATH, self.LOCAL_SAVE_NAME)

    def testGetErrorUrlOpen(self):
        flexmock(urllib2).should_receive('urlopen').and_raise(urllib2.URLError("Error message"))

        with self.assertRaises(DownloaderError):
            self.downloader.attemptDownload()

    def testIOErrorWriteToIsNotNoneVerifyCloseCall(self):
        fileMock = MockHelpers.getOpenMock()
        urlMock = self.__mockUrlOpen()

        urlMock.should_receive('read').and_raise(IOError("error message.")).once().ordered()
        fileMock.should_receive('close').once().ordered()

        with self.assertRaises(DownloaderError):
            self.downloader.attemptDownload()

    def testFinishReadNoErrorVerifyCloseCall(self):
        readString = "bunchofchars"

        fileMock = MockHelpers.getOpenMock()
        urlMock = self.__mockUrlOpen()

        fileMock.should_receive('write').with_args(readString).once().ordered()
        fileMock.should_receive('close').once().ordered()

        urlMock.should_receive('read').and_return("bunchofchars").and_return(None).twice()

        self.downloader.attemptDownload()

    def __mockUrlOpen(self):
        urlMock = flexmock()
        flexmock(urllib2).should_receive('urlopen').and_return(urlMock)
        return urlMock


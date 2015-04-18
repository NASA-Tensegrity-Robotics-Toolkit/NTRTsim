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
from src.download.file_hasher import *

from test.test_helpers.mock_helpers import MockHelpers

class TestFileHasher(unittest.TestCase):

    FILE_HASH = "adc83b19e793491b1c6ea0fd8b46cd9f32e592fc"
    INCORRECT_HASH = "3fdfa9ab969952122126804f9b059633954e859c"
    FILE_PATH = "/home/bruce_wayne/batman_stuff/cute-pictures-of-clark-kent.tar.gz"

    def testIOErrorOnOpenRaisesHasherException(self):
        MockHelpers.raiseIOErrorOnOpen()

        with self.assertRaises(FileHasherException):
            FileHasher(self.FILE_PATH)

    def testIOErrorOnFileReadRaisesHasherException(self):
        fileMock = MockHelpers.getOpenMock()
        fileMock.should_receive('read').and_raise(IOError("Mock error."))
        fileMock.should_receive('close')

        with self.assertRaises(FileHasherException):
            FileHasher(self.FILE_PATH)

    def testHashMatches(self):
        fileHasher = self.__prepMocks()
        self.assertTrue(fileHasher.compareHash(self.FILE_HASH))

    def testHashDoesNotMatch(self):
        fileHasher = self.__prepMocks()
        self.assertFalse(fileHasher.compareHash(self.INCORRECT_HASH))

    def __setHexDigestValue(self, hashToReturn):
        shaOneMock = flexmock()
        shaOneMock.should_receive('update')
        shaOneMock.should_receive('hexdigest').and_return(hashToReturn)

        flexmock(hashlib).should_receive('sha1').and_return(shaOneMock).once()

    def __createFileMock(self):
        fileMock = MockHelpers.getOpenMock()
        fileMock.should_receive('read')
        fileMock.should_receive('close')

    def __prepMocks(self):
        self.__createFileMock()
        self.__setHexDigestValue(self.FILE_HASH)

        return FileHasher(self.FILE_PATH)


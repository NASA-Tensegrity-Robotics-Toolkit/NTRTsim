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
        pass

    def testHashMatches(self):
        self.__createFileMock()
        self.__setHexDigestValue(self.FILE_HASH)

        fileHasher = FileHasher(self.FILE_PATH)

        self.assertTrue(fileHasher.compareHash(self.FILE_HASH))

    def testHashDoesNotMatch(self):
        self.__createFileMock()
        self.__setHexDigestValue(self.FILE_HASH)

        fileHasher = FileHasher(self.FILE_PATH)

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


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
        urlMock = MockHelpers.getUrlLibTwoOpenMock()

        urlMock.should_receive('read').and_raise(IOError("error message.")).once().ordered()
        fileMock.should_receive('close').once().ordered()

        with self.assertRaises(DownloaderError):
            self.downloader.attemptDownload()

    def testFinishReadNoErrorVerifyCloseCall(self):
        readString = "bunchofchars"

        fileMock = MockHelpers.getOpenMock()
        urlMock = MockHelpers.getUrlLibTwoOpenMock()

        fileMock.should_receive('write').with_args(readString).once().ordered()
        fileMock.should_receive('close').once().ordered()

        urlMock.should_receive('read').and_return("bunchofchars").and_return(None).twice()

        self.downloader.attemptDownload()

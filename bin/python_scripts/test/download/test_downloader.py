import unittest
from flexmock import *
from src.download.downloader import *

class TestDownloader(unittest.TestCase):

    URL = "http://www.perryb.ca/bigfile.tar.gz"
    LOCAL_SAVE_PATH = "/home/perry/work/git/NTRTsim/env/downloads"
    LOCAL_SAVE_NAME = "myFile.tar.gz"

    def setUp(self):
        pass

    def testGet404RaiseException(self):
        flexmock(urllib2).should_receive('urlopen').and_raise(urllib2.URLError("Error message"))

        downloader = Downloader(self.URL, self.LOCAL_SAVE_PATH, self.LOCAL_SAVE_NAME)

        with self.assertRaises(DownloaderError):
            downloader.attemptDownload()

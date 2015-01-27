from flexmock import *
from src.utilities.file_utils import FileUtils
import urllib2

class MockHelpers:

    @staticmethod
    def getOpenMock():
        """
        Mocks FileUtils to return a mock, and returns
        that mock from this function so you can run
        expectations on it, etc.
        """
        fileMock = flexmock()
        flexmock(FileUtils).should_receive('open').and_return(fileMock)
        return fileMock

    @staticmethod
    def getUrlLibTwoOpenMock():
        urlMock = flexmock()
        flexmock(urllib2).should_receive('urlopen').and_return(urlMock)
        return urlMock

    @staticmethod
    def raiseIOErrorOnOpen():
        """
        Causes FileUtils.open to raise an IOError, regardless of
        what parameters are passed.
        """
        flexmock(FileUtils).should_receive('open').and_raise(IOError("Fake error."))

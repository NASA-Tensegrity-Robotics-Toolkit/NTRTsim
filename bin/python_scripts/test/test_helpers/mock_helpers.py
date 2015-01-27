from flexmock import *
from src.utilities.file_utils import FileUtils

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
    def raiseIOErrorOnOpen():
        """
        Causes FileUtils.open to raise an IOError, regardless of
        what parameters are passed.
        """
        flexmock(FileUtils).should_receive('open').and_raise(IOError("Fake error."))

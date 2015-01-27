from flexmock import *

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
    def getUrlLib2OpenMock():
        urlMock = flexmock()
        flexmock(urllib2).should_receive('urlopen').and_return(urlMock)
        return urlMock

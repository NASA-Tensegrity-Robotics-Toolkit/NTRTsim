import urllib2

class Downloader:
    """
    Downloader is responsible for, as the name implies, downloading. Downloading packages necessary during installation, specifically.

    While Downloader will throw exceptions in response to download failure (be it a non-responsive IP, hitting a 404, etc), but there are some failures it will not catch. You should verify the hash of any files downloaded with Downloader.
    """

    def __init__(self, targetURL, localSavePath, localSaveName):
        """
        Creates a Downloader instance.

        Parameters

        targetURL: A string containing the target to download.
        localSavePath: A string containing an absolute path to where the resulting file should be saved. Note that this should *not* include
        """
        self.targetURL = targetURL
        self.localSavePath = localSavePath
        self.localSaveName = localSaveName

    def attemptDownload(self):
        try:
            # TODO: For now we're statically specfiying the timeout. Once the configurator is done we'll want to replace this with a timeout from the configurator. This will also necessitate a solution for the configurator during testing.
            urllib2.urlopen(self.targetURL, timeout=5)
        except urllib2.URLError, e:
            raise DownloaderError("Hit a URLError while attempting to urlopen on %s. The error message is '%s'", self.targetURL, e)

class DownloaderError(Exception):
    pass

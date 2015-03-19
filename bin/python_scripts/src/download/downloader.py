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


import urllib2
#TODO: [URGENT] Switch to absolute imports. There are several areas where this will need to be fixed, just grep "from ."
from ..utilities.file_utils import FileUtils

class Downloader:
    """
    #TODO: Set Vim to automatically add line breaks on long comment/docstring lines, then fix all existing comments in the new scripts.
    Downloader is responsible for, as the name implies, downloading. Downloading packages necessary during installation, specifically.

    While Downloader will throw exceptions in response to download failure (be it a non-responsive IP, hitting a 404, etc), but there are some failures it will not catch. You should verify the hash of any files downloaded with Downloader.
    """

    DOWNLOAD_BLOCK_SIZE = 8192

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
        urlConnection = None
        try:
            # TODO: For now we're statically specfiying the timeout. Once the configurator is done we'll want to replace this with a timeout from the configurator. This will also necessitate a solution for the configurator during testing.
            urlConnection = urllib2.urlopen(self.targetURL, timeout=5)
        except urllib2.URLError, e:
            raise DownloaderError("Hit a URLError while attempting to urlopen on %s. The error message is '%s'", self.targetURL, e)

        writeTo = None
        try:
            fullPath = "%s/%s" % (self.localSavePath, self.localSaveName)
            writeTo = FileUtils.open(fullPath, 'wb')

            while True:
                inputBuffer = urlConnection.read(self.DOWNLOAD_BLOCK_SIZE)

                if not inputBuffer:
                    break

                writeTo.write(inputBuffer)

        except IOError, e:
            raise DownloaderError("Hit an IOError while trying to create an output file at %s. The error message is '%s'", e)
        finally:
            if writeTo:
                writeTo.close()

class DownloaderError(Exception):
   pass

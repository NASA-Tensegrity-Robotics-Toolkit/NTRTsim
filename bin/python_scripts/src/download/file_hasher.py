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


from ..utilities.file_utils import FileUtils
import hashlib

class FileHasher:

    def __init__(self, filePath):
        """
        Creates a FileHasher instance.

        Parameters

        filePath: An absolute path to the file to hash.

        Initializing this instance results in the hash being calculated
        (i.e. the hash is calculated at object instantiation, not when
        you call compareHash). As a result, if any failure occurs
        while attempting to determine the target file's hash, a FileHasherException
        will be raised.
        """
        self.filePath = filePath
        self.fileHash = self.__hashFile()

    def compareHash(self, compareTo):
        """
        Returns True if the file pointed to by this
        FileHasher instance has an SHA1 hash which is
        equal to compareTo.

        Returns False otherwise.

        Parameters

        compareTo: A string containing the hash to compare against.
        """
        if compareTo == self.fileHash:
            return True
        else:
            return False

    def __hashFile(self):
        fileToHash = None
        try:
            fileToHash = FileUtils.open(self.filePath)
        except IOError, e:
            #TODO: Move string generation for exceptions into a helper, or have your custom exceptions inherit from a class which can take the error as a 2nd parameter (in addition to the unique prefix string which describes what caused it) and automatically generate the resulting string (the latter seems like a cleaner approach).
            raise FileHasherException("Experienced an error while opening the file path specified for hashing. Attempted to open the file at path %s, which resulted in an error with the message '%s'" % (self.filePath, e))

        #TODO: Should be asserting that file has no handle. Need to be asserting far more
        # often in general. Can strip it out later with -O.

        # Calculate the hash and assign it to an instance var.
        shaOneHash = hashlib.sha1()
        try:
            shaOneHash.update(fileToHash.read())
        except IOError, e:
            raise FileHasherException("Experienced an error while reading from the file at %s while calculating its hash. The specific error message is '%s'" % (self.filePath, e))
        finally:
            fileToHash.close()

        return shaOneHash.hexdigest()

class FileHasherException(Exception):
    pass

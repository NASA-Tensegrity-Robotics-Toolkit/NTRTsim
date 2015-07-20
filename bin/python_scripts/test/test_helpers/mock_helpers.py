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

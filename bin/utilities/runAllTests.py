# Copyright 2012, United States Government, as represented by the
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

import os

# The suffix all test files must have. We match this case-insensitively.
TEST_SUFFIX = "_test"

def isExecutable(filePath):
    return os.path.isfile(filePath) and os.access(filePath, os.X_OK)

def runTest(filePath):
    print "\n*** Running tests executable %s ***\n" % (filePath)
    return os.system(filePath)

# We set this to true if a single test fails.
testFailed = False

for root, subFolders, files in os.walk("."):
    for file in files:
        if file.lower().endswith(TEST_SUFFIX):
            filePath = "%s/%s" % (root, file)
            if isExecutable(filePath):
                exitCode = runTest(filePath)
                if exitCode != 0 and testFailed == False:
                    # This is our first failure
                    testFailed = True

if testFailed:
    exit(1)
else:
    exit(0)

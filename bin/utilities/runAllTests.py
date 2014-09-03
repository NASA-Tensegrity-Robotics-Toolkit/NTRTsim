import os

# The suffix all test files must have. We match this case-insensitively.
TEST_SUFFIX = "_test"

def isExecutable(filePath):
    return os.path.isfile(filePath) and os.access(filePath, os.X_OK)

def runTest(filePath):
    print "Asked to run test: %s" % (filePath)

for root, subFolders, files in os.walk("."):
    for file in files:
        if file.lower().endswith(TEST_SUFFIX):
            filePath = "%s/%s" % (root, file)
            if isExecutable(filePath):
                runTest(filePath)

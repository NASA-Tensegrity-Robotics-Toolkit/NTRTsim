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

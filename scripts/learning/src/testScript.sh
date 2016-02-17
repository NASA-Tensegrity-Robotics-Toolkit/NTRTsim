# Clear previous results
rm -r /media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests

# Run test job
ntrtjob="NTRTJobMaster.py"
file="testGASpec.yaml"
num=4
python $ntrtjob "$file" $num

# Verify test output format
compare="helpersNew/testOutputFormat.py"
fileA="/media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests/monteOut_1_30.json"
fileB="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/GATestsYAMLSeed/monteOut_0_3.json"
python $compare "$fileA" "$fileB"

nnGitIgnore="/media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests/NeuralNet/.gitignore"
gaGitIgnore="/media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests/.gitignore"
printf "*.nnw">$nnGitIgnore
printf "*.json\n*.txt">$gaGitIgnore

# For testing a different terrain
# fileC="/media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests/monteOut_0_0.json"
# fileD="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/GATestsYAMLSeed/monteOut_0_12.json"
# python $compare "$fileD" "$fileC"

# Clear previous results
rm -r /media/leif/Data/Repos/NTRTsim/resources/src/leifrf/StructureTests

# Run test job
ntrtjob="NTRTJobMaster.py"
file="testStructureSpec.yaml"
num=4
python $ntrtjob "$file" $num

# Verify test output format
compare="helpersNew/testOutputFormat.py"
fileA="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/StructureTests/structureOut_0_0.yaml"
fileB="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/3PrismStructureSeed/3Prism.yaml"
python $compare "$fileA" "$fileB"

builderTests="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/StructureTests/.gitignore"
printf "*.yaml\n*.json\n*.txt">$builderTests

# For testing a different terrain
# fileC="/media/leif/Data/Repos/NTRTsim/resources/src/bmirletz/GATests/monteOut_0_0.json"
# fileD="/media/leif/Data/Repos/NTRTsim/resources/src/leifrf/GATestsYAMLSeed/monteOut_0_12.json"
# python $compare "$fileD" "$fileC"

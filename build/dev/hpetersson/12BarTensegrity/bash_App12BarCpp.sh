#!/bin/bash

echo "Bash script is running"

nEpisodes=1

inputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/InputRandomMatlab/actions_d_205.csv" 

outputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/gen_d_filetest_5.csv"
	

for i in `seq 0 $nEpisodes`;
do
	./App12BarCpp $i "$inputPath" "$outputPath"
	echo $i
#	echo "$inputPath"
#	echo "$outputPath"
done



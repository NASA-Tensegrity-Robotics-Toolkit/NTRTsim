#!/bin/bash

echo "Bash script is running"

nEpisodes=2

inputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/InputRandomMatlab/randomactions_d_3_2018-06-13-10-39.csv" 

outputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/gen_d_filetest_1.csv"
	

for i in `seq 0 $nEpisodes`;
do
	./App12BarCpp $i "$inputPath" "$outputPath"
	echo $i
#	echo "$inputPath"
#	echo "$outputPath"
done



#!/bin/bash

echo "Bash script is running"

nEpisodes=2

inputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/ground12BarTensegrity/InputActions/randappact_z_100_2018-06-28-16-29.csv" 

outputPath="/home/hannah/Projects/NTRTsim/src/dev/hpetersson/ground12BarTensegrity/outputFiles/gen_z_test.csv"
	

for i in `seq 0 $nEpisodes`;
do
	./App12BarGround $i "$inputPath" "$outputPath"
	echo $i
#	echo "$inputPath"
#	echo "$outputPath"
done



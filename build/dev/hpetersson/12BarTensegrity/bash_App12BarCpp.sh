#!/bin/bash

echo "Bash script is running"

nEpisodes=2

for i in `seq 0 $nEpisodes`;
do
	./App12BarCpp $i
#"/home/hannah/Projects/NTRTsim/src/dev/hpetersson/12BarTensegrity/outputFiles/gen_d_filetest.csv"
	echo $i
done



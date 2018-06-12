#!/bin/bash

echo "Bash script is running"

nEpisodes=10

for i in `seq 0 $nEpisodes`;
do
	./App12BarCpp $i
	echo $i
done



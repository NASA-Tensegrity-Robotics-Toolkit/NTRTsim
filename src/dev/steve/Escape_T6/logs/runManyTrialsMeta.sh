#!/bin/sh
max=80 # num of lines in trial_7/bestParametersNoOutliers.dat
for i in `seq 1 $max`
do
    echo "Meta Trial $i"
    ~/NTRTsim/src/dev/steve/Escape_T6/logs/runManyTrials.sh ;
done

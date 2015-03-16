#!/bin/bash

if [ $# -lt 1 ]; then
    trialNum=0
else
    trialNum=$1
fi

if [ $trialNum -lt 8 ] && [ $trialNum -ne 0 ]; then
    echo "Already ran trials 1-7, pick another number"
    exit
fi

trialDir="logs/mechtest/trial_$trialNum"

mkdir -p $trialDir

echo 'Run 1: All Strings'
./AppDuCTTMechTest -c -y 5 -T 0 -t 60 2>$trialDir/run1_allstrings.csv
echo 'Run 2: Top Vertical Strings'
./AppDuCTTMechTest -c -y 5 -T 1 -t 60 2>$trialDir/run2_top_vertstrings.csv
echo 'Run 3: Bottom Vertical Strings'
./AppDuCTTMechTest -c -y 5 -T 2 -t 60 2>$trialDir/run3_bottom_vertstrings.csv

zip $trialDir/DuCTT_MechTest_Trial$trialNum.zip $trialDir/run*strings.csv

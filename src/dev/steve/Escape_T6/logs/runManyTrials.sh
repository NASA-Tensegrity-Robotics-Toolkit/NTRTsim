#!/bin/bash
max=10
for i in `seq 1 $max`
do
    echo "Trial $i"
    cd ~/NTRTsim/src/dev/steve/Escape_T6 ;
    ~/NTRTsim/build/dev/steve/AppEscape_T6 ;
    cd -
done

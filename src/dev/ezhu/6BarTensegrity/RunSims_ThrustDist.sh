#!/bin/bash

# echo ${BASH_VERSION}

cd ~/NTRTsim/build/dev/ezhu/6BarTensegrity/
rm -r ./data
mkdir data

interval=$[1]
min_vel=$[10]

# interval=$[2]
# min_angle=$[-10]
# max_angle=$[10]

# ./App6Bar $[5] $[7] "a.txt"

# yaw=$[0]
# for i in {0..36}
# do
# 	pitch=$[$i*$interval+$min_angle]
# 	for j in {0..36}
# 	do
# 		roll=$[$j*$interval+$min_angle]
# 		log_name="${pitch}P${roll}R_Response.txt"
# 		./App6Bar $yaw $pitch $roll $log_name
# 		# echo $pitch $roll $log_name
# 	done
# done

vz=$[0]
for i in {0..40}
# for i in {0..1}
do
	vx=$[$i*$interval+$min_vel]
	vy=$[$i*$interval+$min_vel]
	log_name="./data/${vx}V_Response.csv"
	./App6Bar 0 0 0 $vx $vy $vz $log_name
done

zip -r data.zip ./data
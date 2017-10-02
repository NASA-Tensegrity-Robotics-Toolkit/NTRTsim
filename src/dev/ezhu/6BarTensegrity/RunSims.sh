#!/bin/bash

# echo ${BASH_VERSION}

cd ~/NTRTsim/build/dev/ezhu/6BarTensegrity/
rm -r ./data
mkdir data

interval=$[1]
min_angle=$[0]
max_angle=$[180]

# interval=$[2]
# min_angle=$[-10]
# max_angle=$[10]

num_pts=$[($max_angle-$min_angle)/$interval]
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

# pitch=$[0]
# roll=$[0]
# for i in {0..180}
# # for i in {0..1}
# do
# 	yaw=$[$i*$interval+$min_angle]
# 	log_name="./data/${yaw}Y_Response.csv"
# 	./App6Bar $yaw $pitch $roll $log_name
# done

for i in {0..500}
do
	log_name="./data/${i}_Response.csv"
	./App6Bar $log_name
done

zip -r data.zip ./data
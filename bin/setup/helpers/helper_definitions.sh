#!/bin/bash

# Copyright © 2012, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# All rights reserved.
# 
# The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
# under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0.
# 
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

# Constants
TRUE=0  # Yes, TRUE is 0 (e.g., no errors)
FALSE=1 # Ditto, FALSE is non-zero

script_name=$(basename $0)

# This runs make with either the total number of cores
# specified by the user in build.conf, or using the maximum
# number of cores in the user's system.
function set_multicore_make()
{
    source_conf "build.conf"
    if [ -n "$MAX_BUILD_CORES" ]; then
        max_cores=$MAX_BUILD_CORES
    elif [ -f /proc/cpuinfo ]; then
        # Linux
        max_cores=`grep -c ^processor /proc/cpuinfo`
    else
        # Mac
        max_cores=`sysctl -n hw.ncpu`
    fi

    shopt -s expand_aliases
    alias make="make -j$max_cores"
}

if [[ "$script_name" != "setup.sh" ]]; then
    set_multicore_make
fi

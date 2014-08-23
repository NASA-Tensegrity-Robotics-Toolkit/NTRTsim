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

# Purpose: Common setup code
# Date:    2014-08-18

function source_conf()
{
    conf_file_name="$CONF_DIR/$1"
    if [ ! -f "$conf_file_name" ]; then
        echo "Missing $conf_file_name. Please fix this and try again."
        exit 1
    fi
    source	"$conf_file_name"
}

# Deteremine if a string contains a substring
# Usage: tf=$(str_contains "my string" "substring")
function str_contains()
{
    string="$1"
    substring="$2"
    if test "${string#*$substring}" != "$string"
    then
        return 0    # $substring is in $string
    else
        return 1    # $substring is not in $string
    fi
}

# Get the relative path between two absolute paths
# Usage: rel=$(get_relative_path /absolute/path/one /absolute/path/two)
function get_relative_path()
{
    source=$1
    target=$2

    common_part=$source
    back=
    while [ "${target#$common_part}" = "${target}" ]; do
        common_part=$(dirname $common_part)
        back="../${back}"
    done

    echo ${back}${target#$common_part/}
}

# Count the number of files matching the given pattern
# Usage: e.g. n_files=$(count_files "/path/with/file/matching/pattern/*.*")
# Note: the 'echo `command`' is there to remove leading spaces that wc adds
function count_files()
{
    pattern=$1
    if [ "$pattern" == "" ]; then
        pattern="*"
    fi
    # Count the number of files
    echo `ls -a $pattern 2>/dev/null | wc -l`
}



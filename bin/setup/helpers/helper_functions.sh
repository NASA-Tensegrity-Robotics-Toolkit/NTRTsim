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

function ensure_install_prefix_writable()
{
    touch "$1/tensegrity.deleteme" 2>/dev/null \
        || { echo "Install prefix '$1' is not writable -- please use sudo or execute as root."; exit 1; }
    rm "$1/tensegrity.deleteme"
}


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

# Count the number of occurrences of a specific selection string.
# For example, calling this function with "$BOOST_INSTALL_PREFIX/lib/libboost*"
# as a parameter would tell you whether any files match that selection string.
function check_package_installed()
{
    count_libs=$(count_files $1)
    if [ "$count_libs" == "0" ]; then
        return $FALSE
    fi
    return $TRUE
}

# Returns TRUE if the directory exists,
# FALSE otherwise. Assumes the script that calls
# the function has included helper_definitions.sh
# (that's where TRUE and FALSE are defined).
function check_directory_exists()
{
    if [ -d $1 ]; then
        return $TRUE
    fi
    return $FALSE
}

# If the provided directory does not exist, it creates
# it using mkdir with the -p flag (in order to create
# all intermediate directories as well.
# If creation of the directory fails, an informative
# message is echoed and we exit with an exit code of 1.
function create_directory_if_noexist()
{
    if ! check_directory_exists $1; then
        mkdir -p $1 || { echo "Failed while attempting to create directory: ${1}. This is a fatal error. Are you sure you have write access to that directory?"; exit 1; }
    fi
}

# Returns TRUE (0) if the file exists,
# FALSE (1) otherwise.
function check_file_exists()
{
    if [ -d $1 ]; then
        return $TRUE
    fi
    return $FALSE
}

# Allow user to select from a set of options and return the selected option
# usage: result=$(read_options "Is the default option capitalized? (yes, no, abort)" ("Y" "n" "a") "Y")
function read_options()
{
    message=$1
    options=$2
    default=$3

    # Assemble the options
    str_opts=$(printf "/%s" "${options[@]}")
    str_opts="[${str_opts:1}]"
    read -p "$message $str_opts " input
    if [[ "$input" == "" ]]; then
        input="$default"
    fi
    input=`echo $input|tr [a-z] [A-Z]`

    # TODO: Check options, make sure that a proper one was selected (maybe)

    echo "$input"
}

# Read a line of text, returning the default if user hits enter.
# usage: result=$(read_text "Is this the prompt" "yes, I believe it is.")
function read_text()
{
    message=$1
    default=$2

    read -p "$message: " input
    if [[ "$input" == "" ]]; then
        input="$default"
    fi
    echo "$input"
}

# Returns TRUE (0) if the user has the command
# provided in the path. Returns FALSE (1) otherwise.
function has_command()
{ 
    command -v $1 > /dev/null 2>&1 || { return $FALSE; } 
} 

# This function requires two parameters.
# 
# $1 should include the path (either absolute or relative) to
# the file that needs its md5 checked.
#
# $2 should be the expected md5 output, this includes both
# the hash and the file's name, as is generated by a md5sum utility.
#
# If MD5 hash verification fails, an error is printed and the program exits
# with an error code of 1. MD5 hash verification is considered to have failed
# if, of course, the hash is invalid, but also if the specified file for verification
# can not be found.
# 
# If MD5 hash verification succeeds, a message is printed indicating as much
# and the function returns.
function verify_md5()
{
    if [ ! -n $MD5_SUM_BINARY ]; then
        echo "=== SKIPPING MD5 VERIFICATION ==="
        echo "Skipping verification of $1 as MD5 verification is disabled."
        echo "=================================="
        return
    fi

    if [ ! -f $1 ]; then
        echo "==== MD5 VERIFICATION FAILURE ==="
        echo "Could not find file $1 to generate MD5."
        echo ""
        echo "This likely means that the download failed. If you received this error message"
        echo "in spite of the file existing, please report it as a bug on GitHub."
        echo ""
        echo "Setup will now exit."
        echo "================================="
        exit 1
    fi

    md5_raw_output=`$MD5_SUM_BINARY $1`
    md5output=
    # Extract the hash only.
    for word in $md5_raw_output
    do
        md5output=$word
        break
    done

    if [ "$md5output" == "$2" ]; then
        echo "=== MD5 HASH VERIFIED ==="
        echo "Completed MD5 verification on $1"
        echo ""
        echo "Expected: $2"
        echo "Received: $md5output"
        echo ""
        echo "File successfully verified."
        echo "========================="
        return 
    fi

    echo "====  MD5 VERIFICATION FAILURE ===="
    echo "MD5 VERIFICATION FAILED."
    echo ""
    echo "Completed MD5 verification on $1"
    echo ""
    echo "Expected: $2"
    echo "Received: $md5output"
    echo ""
    echo "This likely means that the download failed (either it timed out, or the"
    echo "file we downloaded is incorrect)."
    echo ""
    echo "If you believe you received this message in error, please report it as a bug on GitHub."
    echo "In the meantime you can, if desired, disable md5sum checking in general.conf. Only do"
    echo "this if you are absolutely certain this message is erroneous."
    echo ""
    echo "Setup will now exit."
    echo "==================================="

    exit 1
}

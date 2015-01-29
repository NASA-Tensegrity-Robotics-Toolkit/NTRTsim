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

# Downloads the file located at the first argument to
# the local path specified by the second argument.
# 
# This call exits with a return code of 1 if curl returns
# a non-zero error code.
# 
# It's important to note that curl is being called with the -f
# flag in this case. So curl will return a non-zero exit code
# in any case where an HTTP return > 400 is returned. As a result
# this should gracefully handle cases like error 404, as opposed to
# simply saving the 404 page.
function download_file()
{
    download_url=$1
    save_to=$2

    # We'll manually set our connection time out. No need to put this
    # in a conf file. No conceivable reason anyone will want granular
    # control over this.
    connect_timeout=30

    curl -k -f --connect-timeout $connect_timeout -L "$download_url" > "$save_to" || 
    { 
        rm "$save_to"
        echo "======== DOWNLOAD FAILURE ========="
        echo "Encountered a failure while downloading:"
        echo ""
        echo "Target URL: $download_url"
        echo "Local URL: $save_to"
        echo ""
        echo "Exiting now."
        echo "======== DOWNLOAD FAILURE ========="
        exit 1 
    }
}

# Creates a symlink from link_location to link_target.
# If the symlink already exists, we verify that it's a
# valid symlink (that is, we verify that it points to a valid target).
# If a symlink is invalid, we delete it and attempt to re-create it.
# If symlink creation fails (after creating the symlink it points to
# an invalid location) we exit with an error code of 1.
#
# NOTE: This function should only be used when you're making a direct
# symlink to a *single* file. If you're attempting to create a symlink
# using a wildcard in order to hit every file in a directory
# (e.g. ln -s /usr/bin/* .) you should *NOT* use this function. It will fail.
function create_exist_symlink()
{
    link_target=$1
    link_location=$2

    # Check that the symbolic link exists (may not be valid).
    if [ -h $link_location ]; then
        # Now check whether the symlink is valid.
        if [ -e $link_location ]; then
            return
        else
            #Delete the symlink. It's broken.
            rm $link_location 
        fi
    fi

    # At this point there is no symbolic link file. We need to create it.
    ln -s $link_target $link_location

    # Now verify that our symlink exists and points to a valid location. 
    if [ ! -e $link_location ]; then
        echo "Failure while creating symlink from $link_location to $link_target. This is a fatal error."
        exit 1
    fi
}

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

# Purpose: Install script for everything related to the tensegrity repository
# Author:  Ryan Adams, Perry Bhandal

##############################################################################
#                       Services Configuration                               #
##############################################################################
# Add the relative path from your current directory to the bash services folder
# so we can import all helper scripts. If this script is operating from the
# root directory
pushd "services/bash/" > /dev/null

if [ ! -f "helper_functions.sh" ]; then
    echo "Could not find helper_functions.sh. Are we in the bash services folder?"
    exit 1;
fi

# Import our common files
source "helper_functions.sh"
source "helper_paths.sh"

# Get out of the bash services folder.
popd > /dev/null
##############################################################################

### Begin universal configuration 
SCRIPT_PATH="`dirname \"$0\"`"                  # relative
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"  # absolutized and normalized
setup_dir="$SCRIPT_PATH/bin/setup"
conf_dir="$SCRIPT_PATH/conf"
base_dir="`( cd \"$setup_dir/\" && pwd )`"   # Required for init scripts
### End universal configuration

### Begin script configuration 
env_name='env'
CONF_FILES=("general.conf" "boost.conf" "bullet.conf") 
### End script configuration

function banner() 
{
    echo ""
    echo "== Setup ============"
}

function init_config()
{

    if [ -f "$conf_dir/install.conf" ]; then
        echo "Your conf directory contains install.conf, which has been deprecated. Please delete install.conf and run set up again. See issue 21 for more details."
        exit 1
    fi

    echo "- Starting configuration"

    ## Determine if any conf files are missing
    to_create=()

    for file_name in "${CONF_FILES[@]}"
    do
        if [ ! -f "$conf_dir/$file_name" ]; then
            to_create+=($file_name)
        fi
    done

    if [ "${#to_create}" -eq 0 ]; then
        source "$conf_dir/general.conf"
        return
    fi

    message="One or more package files do not exist. Would you like setup to create them now?"
    options=("Y" "n")
    default="Y"
    result=$(read_options "$message" $options $default)
    if [ "$result" == "Y" ]; then
        for file_name in "${to_create[@]}"
        do
            echo "Creating conf/$file_name"
            cp "$conf_dir/default/${file_name}.default" "$conf_dir/$file_name"  || { echo "Could not find default conf file ${file_name}.default in conf/default -- exiting now."; exit 1; }  
        done
        echo "All missing package configuration files have been created in conf/*. Please edit as needed and then re-run setup.sh."
    else
        echo "Please create all necessary conf files before running set up. See conf/default/* for a list of the needed files, as well as a base configuration."
    fi

    exit 0
}

function init_scripts()
{
    # Make sure permissions are correct, etc.
    pushd "$setup_dir" > /dev/null
    chmod a+x *
    popd > /dev/null
}

function run_setupscript()
{
    script_name=$1
    full_name=$2
    echo ""
    echo "Initializing ${2}..."
    "$setup_dir/setup_${script_name}.sh" || { echo "$full_name initialization failed -- exiting now."; exit 1; }
}

# Set a variable in the install.conf configuration file
function set_config_var()
{
    var=$1
    value=$2
    echo "var: $var; value: $value"
    sed "s,$var=.*,$var=\"$value\",g" "$setup_dir/install.conf" > "$setup_dir/install.conf.tmp"
    mv "$setup_dir/install.conf.tmp" "$setup_dir/install.conf"

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

# TEST FUNCTIONS

function test_read_options()
{
    message="Do you want to test read_options?"
    options=("Y" "n" "a")
    default="Y"
    output=$(read_options "$message" $options $default)
    echo "read_options() returned '$output'"
}

function test_read_text()
{
    message="Do you want to test read_text?"
    default="I'd rather test the default."
    output=$(read_text "$message" "$default")
    echo "read_text() returned '$output'"
}

function test_relative_path()
{
    a="/this/is/an/absolute/path"
    b="/this/is/the/target/path"
    rel=$(get_relative_path "$a" "$b")
    echo "relative path is $rel (should be '../../../the/target/path')"
}

# EXECUTION

banner
init_config
init_scripts

run_setupscript "env" "Env directory"
run_setupscript "cmake" "CMake"
run_setupscript "bullet" "Bullet Physics Library"
run_setupscript "boost" "Boost"

echo ""
echo "Setup Complete!"
echo ""
echo "* Next Steps: Use bin/build.sh to build your code. The results will be placed under the 'build' directory."
echo ""

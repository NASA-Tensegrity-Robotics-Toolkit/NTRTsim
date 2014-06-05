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

# Purpose: Boost library Setup
# Date:    2013-05-01

###############################
# Configuration
local_setup_path="`dirname \"$0\"`"                # relative
base_dir="`( cd \"$local_setup_path/../../\" && pwd )`"  # absolutized and normalized
install_conf_file="$base_dir/conf/install.conf"
if [ ! -f "$install_conf_file" ]; then
    echo "Missing install.conf ($install_conf_file). Please fix this and try again."
    exit 1
fi
source "$install_conf_file"
###############################


# Variables
boost_pkg=`echo $BOOST_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Constants
TRUE=0  # Yes, TRUE is 0 (e.g., no errors)
FALSE=1 # Ditto, FALSE is non-zero

function ensure_install_prefix_writable() {
    touch "$BOOST_INSTALL_PREFIX/tensegrity.deleteme" 2>/dev/null \
        || { echo "Install prefix '$BOOST_INSTALL_PREFIX' is not writable -- please use sudo or execute as root."; exit 1; }
    rm "$BOOST_INSTALL_PREFIX/tensegrity.deleteme"
}

# Check if the package is already installed in the location specified in install.conf 
# TODO: Check to make sure that the header files are also installed properly.
function check_boost_installed() {
    count_boost_libs=$(count_files "$BOOST_INSTALL_PREFIX/lib/libboost*")
    if [ "$count_boost_libs" == "0" ]; then
        return $FALSE
    fi
    return $TRUE
}

# Check to see if boost has been built already
function check_boost_built() {
    # Check for the 'stage' directory that's created when boost is built    
    if [ -d "$BOOST_BUILD_DIR/stage" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Check to see if boost has been unpacked
function check_boost_unpacked() {
    # Check for the 'boost' subdirectory that's present when boost is unpacked
    if [ -d "$BOOST_PACKAGE_DIR/boost" ]; then
        return $TRUE
    fi
    return $FALSE
}

# Determine if the package exists under env/downloads
function check_boost_downloaded() {
    if [ -f "$downloads_dir/$boost_pkg" ]; then
        return $TRUE
    fi
    return $FALSE
}


# Download the package to env/downloads
function download_boost() {

    boost_pkg_path="$downloads_dir/$boost_pkg"

    if [ -f "$boost_pkg_path" ]; then
        echo "- Boost package already exists ('$boost_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $boost_pkg to $boost_pkg_path"
    curl -k -L "$BOOST_URL" > "$boost_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_boost() {
    # Create directory and unpack
    if [ -d "$BOOST_BUILD_DIR" ]; then
        echo "- Boost is already unpacked to '$BOOST_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking boost to $BOOST_BUILD_DIR (this may take a minute...)"
    if [ ! -d "$BOOST_BUILD_DIR" ]; then
        # TODO: Do we need to remove the dir if it already exists?
        mkdir -p "$BOOST_BUILD_DIR"
    fi

    # Unzip
    pushd "$BOOST_BUILD_DIR" > /dev/null
    tar xf "$downloads_dir/$boost_pkg" --strip 1 
    popd > /dev/null

}

# Build the package under the build directory specified in in install.conf
function build_boost() {
    
    echo "- Building Boost under $BOOST_BUILD_DIR"
    pushd "$BOOST_BUILD_DIR" > /dev/null
    
    if [ -d "stage" ]; then
        echo "- Boost is already built in '$BOOST_BUILD_DIR' -- skipping."
        return
    fi

    # Perform the build
    #./bootstrap.sh --prefix="$BOOST_INSTALL_PREFIX" --with-libraries=system || { echo "Boost bootstrap failed."; exit 1; } # Lite
    ./bootstrap.sh --prefix="$BOOST_INSTALL_PREFIX" || { echo "Boost bootstrap failed."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_boost() {
    
    echo "- Installing Boost under $BOOST_INSTALL_PREFIX"

    pushd "$BOOST_BUILD_DIR" > /dev/null
    ./b2 install || { echo "Install failed"; exit 1; }
    popd > /dev/null
            
}

# Create symlinks under env for building our applications and IDE integration
function env_link_boost() {

    # Build
    pushd "$env_dir/build" > /dev/null
    rm boost 2>/dev/null  # This will fail if 'boost' is a directory, which is what we want.
    if [ -d "$BOOST_BUILD_DIR" ]; then  # If we built boost (as opposed to installing it another way)...
        
        # If we're building under env, use a relative link; otherwise use an absolute one.
        if str_contains "$BOOST_BUILD_DIR" "$env_dir"; then
            current_pwd=`pwd`
            rel_path=$(get_relative_path "$current_pwd" "$BOOST_BUILD_DIR" )
            ln -s "$rel_path" boost
        else
            ln -s "$BOOST_BUILD_DIR" boost  # this links directly to the most recent build...
        fi

    fi
    popd > /dev/null
    
    # Header Files
    pushd "$env_dir/include" > /dev/null
    if [ ! -d "boost" ]; then  # We may have built boost here, so only create a symlink if not
        rm boost 2>/dev/null
        ln -s "$BOOST_INSTALL_PREFIX/include/boost" boost
    fi
    popd > /dev/null

}

# Allow user to select from a set of options and return the selected option
# usage: myResult=$(read_options "Is the default option capitalized? (yes, no, abort)" ("Y" "n" "a") "Y")
function read_options() {
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

# Deteremine if a string contains a substring
# Usage: tf=$(str_contains "my string" "substring")
function str_contains() {
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
function get_relative_path() {
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
function count_files {
    pattern=$1
    if [ "$pattern" == "" ]; then
        pattern="*"
    fi
    # Count the number of files
    echo `ls -a $pattern 2>/dev/null | wc -l`
}


function main() {
    
    ensure_install_prefix_writable
    
    if check_boost_installed; then
        echo "- Boost is installed under prefix $BOOST_INSTALL_PREFIX -- skipping."
        env_link_boost
        return
    fi
    
    if check_boost_built; then
        echo "- Boost is already built under $BOOST_BUILD_DIR -- skipping."
        install_boost
        env_link_boost
        return
    fi
    
    if check_boost_unpacked; then
        echo "- Boost is already unpacked to $BOOST_BUILD_DIR -- skipping."
        build_boost
        install_boost
        env_link_boost
        return
    fi
    
    if check_boost_downloaded; then
        echo "- Boost package already exists under env/downloads -- skipping download."
        unpack_boost
        build_boost
        install_boost
        env_link_boost
        return
    fi
    
    # If we haven't returned by now, we have to do everything
    download_boost
    unpack_boost
    build_boost
    install_boost
    env_link_boost

}


main

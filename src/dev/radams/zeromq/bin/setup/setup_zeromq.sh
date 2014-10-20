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

# Purpose: ZeroMQ setup
# Author:  Ryan Adams
# Date:    2014-10-08

# @todo: zeromq installs .h files directly under env/include. 
# That's not the behavior of other packages, but it may be ok. 
# It seems to be the only ./configure / make / make install that we do...


##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"

# Add the relative path from this script to the helpers folder.

# @todo: Change this to work with standard setup locations (this will work for src/dev/radams)
# pushd "${SCRIPT_PATH}/helpers/" > /dev/null  # for standard location setup
pushd "${SCRIPT_PATH}/../../../../../../bin/setup/helpers/" > /dev/null

if [ ! -f "helper_functions.sh" ]; then
    echo "Could not find helper_functions.sh. Are we in the bash helpers folder?"
    exit 1;
fi

# Import our common files
source "helper_functions.sh"
source "helper_paths.sh"
source "helper_definitions.sh"

# Get out of the bash helpers folder.
popd > /dev/null
##############################################################################
#                          END DO NOT MODIFY                                 #
##############################################################################

#Source this package's configuration
source_conf "general.conf"
# source_conf "zeromq.conf"  # for standard setup
source "../../conf/zeromq.conf"  # for dev setup

# Variables
zeromq_pkg=`echo $ZEROMQ_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if zeromq has been built already
function check_zeromq_built()
{
    # Check for a library that's created when zeromq is built   
    # @todo: make this work for zeromq
    fname=$(find "$ZEROMQ_INSTALL_PREFIX/lib/" -iname libzmq* 2>/dev/null)
    if [ -f "$fname" ]; then
        return $TRUE
    fi
    return $FALSE
}


# Download the package to env/downloads
function download_zeromq()
{

    zeromq_pkg_path="$DOWNLOADS_DIR/$zeromq_pkg"

    if [ -f "$zeromq_pkg_path" ]; then
        echo "- ZeroMQ package already exists ('$zeromq_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $zeromq_pkg to $zeromq_pkg_path"
    download_file "$ZEROMQ_URL" "$zeromq_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_zeromq()
{
    # Create directory and unpack
    if check_directory_exists "$ZEROMQ_BUILD_DIR"; then
        echo "- ZeroMQ is already unpacked to '$ZEROMQ_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking zeromq to $ZEROMQ_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $ZEROMQ_BUILD_DIR

    # Unzip
    pushd "$ZEROMQ_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$zeromq_pkg" --strip 1 || { echo "- ERROR: Failed to unpack ZeroMQ."; exit 1; }
    popd > /dev/null
}

# Build the package under the build directory specified in in zeromq.conf
function build_zeromq()
{

    echo "- Building ZeroMQ under $ZEROMQ_BUILD_DIR"
    pushd "$ZEROMQ_BUILD_DIR" > /dev/null

    # Perform the build
    ./configure --prefix="$ZEROMQ_INSTALL_PREFIX"
    
    make || { echo "- ERROR: ZeroMQ build failed."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_zeromq()
{

    echo "- Installing ZeroMQ under $ZEROMQ_INSTALL_PREFIX"

    pushd "$ZEROMQ_BUILD_DIR" > /dev/null

    make install || { echo "- ERROR: ZeroMQ 'make install' failed."; exit 1; }

    popd > /dev/null
}

# Create symlinks under env for building our applications and IDE integration
function env_link_zeromq()
{

    # Build
    pushd "$ENV_DIR/build" > /dev/null
    rm zeromq 2>/dev/null   # Note: this will fail if 'zeromq' is a directory, which is what we want.

    # If we're building under env, use a relative path for the link; otherwise use an absolute one.
    if str_contains "$ZEROMQ_BUILD_DIR" "$ENV_DIR"; then
        current_pwd=`pwd`
        rel_path=$(get_relative_path "$current_pwd" "$ZEROMQ_BUILD_DIR" )
        create_exist_symlink "$rel_path" zeromq
    else
        create_exist_symlink "$ZEROMQ_BUILD_DIR" zeromq  # this links directly to the most recent build...
    fi

    popd > /dev/null

    # Header Files
    pushd "$ENV_DIR/include" > /dev/null
    if [ ! -d "zeromq" ]; then  # We may have built here, so only create a symlink if not
        rm zeromq 2>/dev/null
        create_exist_symlink "$ZEROMQ_INSTALL_PREFIX/include/zeromq" zeromq
    fi
    popd > /dev/null

}

function main()
{

    ensure_install_prefix_writable $ZEROMQ_INSTALL_PREFIX

    # Testing only
    download_zeromq
    unpack_zeromq
    build_zeromq
    install_zeromq
    # env_link_zeromq  # @todo: no need for this?

    exit 1
    
    
    # @todo: does this work? What actually gets installed under lib?
    if check_package_installed "$ZEROMQ_INSTALL_PREFIX/lib/libzmq*"; then
        echo "- ZeroMQ is installed under prefix $ZEROMQ_INSTALL_PREFIX -- skipping."
        env_link_zeromq
        return
    fi

    if check_zeromq_built; then
        echo "- ZeroMQ is already built under $ZEROMQ_BUILD_DIR -- skipping."
        install_zeromq
        env_link_zeromq
        return
    fi

    # @todo: make this work for zeromq
    if check_file_exists "$ZEROMQ_PACKAGE_DIR/CMakeLists.txt"; then
        echo "- ZeroMQ is already unpacked to $ZEROMQ_BUILD_DIR -- skipping."
        build_zeromq
        install_zeromq
        env_link_zeromq
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$zeromq_pkg"; then
        echo "- ZeroMQ package already exists under env/downloads -- skipping download."
        unpack_zeromq
        build_zeromq
        install_zeromq
        env_link_zeromq
        return
    fi




    # If we haven't returned by now, we have to do everything
    download_zeromq
    unpack_zeromq
    build_zeromq
    install_zeromq
    env_link_zeromq

}

main

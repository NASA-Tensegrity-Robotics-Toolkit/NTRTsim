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

# @todo: zeromq installs .h files directly under env/include -- is this ok? 
# Note: this is also the only 'standard' package (configure/make/make install)
# that we currently have. In any case, it works...


##############################################################################
#                         START DO NOT MODIFY                                #
##############################################################################
SCRIPT_PATH="`dirname \"$0\"`"
SCRIPT_PATH="`( cd \"$SCRIPT_PATH\" && pwd )`"

# Add the relative path from this script to the helpers folder.
pushd "${SCRIPT_PATH}/helpers/" > /dev/null

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
source_conf "zeromq.conf"

# Variables
zeromq_pkg=`echo $ZEROMQ_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if zeromq has been built already
function check_zeromq_built()
{
    # Check for a file that's created when zeromq is configured   
    fname=$(find "$ZEROMQ_BUILD_DIR" -iname config.status 2>/dev/null)
    if [ ! -f "$fname" ]; then
        return $FALSE
    fi
    
    # Check for a file that's created when zeromq is made
    fname=$(find "$ZEROMQ_BUILD_DIR/src" -iname *.o 2>/dev/null)
    if [ ! -f "$fname" ]; then
        return $FALSE
    fi
    
    return $TRUE
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


# Download the package to env/downloads
function download_cppzmq()
{

    zeromq_cppzmq_path="$ZEROMQ_CPPZMQ_DOWNLOADS_DIR/zmq.hpp"

    if [ -f "$zeromq_cppzmq_path" ]; then
        echo "- ZeroMQ C++ Binding file already exists ('$zeromq_cppzmq_path') -- skipping download."
        return
    fi

    if [ ! -d "$ZEROMQ_CPPZMQ_DOWNLOADS_DIR" ]; then
        /bin/mkdir -p "$ZEROMQ_CPPZMQ_DOWNLOADS_DIR"
    fi

    echo "Downloading ZeroMQ C++ Bindings to $ZEROMQ_CPPZMQ_DOWNLOADS_DIR"
    download_file "$ZEROMQ_CPPZMQ_URL" "$ZEROMQ_CPPZMQ_DOWNLOADS_DIR/zmq.hpp"
}

function install_cppzmq()
{
    if [ -f "$ZEROMQ_INSTALL_PREFIX/includes/zmq.hpp" ]; then
        echo "- ZeroMQ C++ Bindings already exist ('$ZEROMQ_INSTALL_PREFIX/includes/zmq.hpp') -- skipping install."
        return
    fi
    
    echo "- Installing ZeroMQ C++ Bindings under $ZEROMQ_INSTALL_PREFIX"
    
    /bin/cp "$ZEROMQ_CPPZMQ_DOWNLOADS_DIR/zmq.hpp" "$ZEROMQ_INSTALL_PREFIX/include/"

}



function main()
{

    ensure_install_prefix_writable $ZEROMQ_INSTALL_PREFIX
    
    if check_package_installed "$ZEROMQ_INSTALL_PREFIX/lib/libzmq*"; then
        echo "- ZeroMQ is installed under prefix $ZEROMQ_INSTALL_PREFIX -- skipping."
        download_cppzmq
        install_cppzmq
        return
    fi

    if check_zeromq_built; then
        echo "- ZeroMQ is already built under $ZEROMQ_BUILD_DIR -- skipping."
        install_zeromq
        download_cppzmq
        install_cppzmq
        return
    fi

    if check_file_exists "$ZEROMQ_PACKAGE_DIR/configure"; then
        echo "- ZeroMQ is already unpacked to $ZEROMQ_BUILD_DIR -- skipping."
        build_zeromq
        install_zeromq
        download_cppzmq
        install_cppzmq
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$zeromq_pkg"; then
        echo "- ZeroMQ package already exists under env/downloads -- skipping download."
        unpack_zeromq
        build_zeromq
        install_zeromq
        download_cppzmq
        install_cppzmq
        return
    fi



    # If we haven't returned by now, we have to do everything
    download_zeromq
    unpack_zeromq
    build_zeromq
    install_zeromq

    download_cppzmq
    install_cppzmq

}

main

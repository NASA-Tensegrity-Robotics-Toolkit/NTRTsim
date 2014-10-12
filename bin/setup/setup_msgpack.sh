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

# Purpose: MessagePack setup
# Author:  Ryan Adams
# Date:    2014-10-08

# @todo: msgpack installs .h files directly under env/include -- is this ok? 
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
source_conf "msgpack.conf"

# Variables
msgpack_pkg=`echo $MSGPACK_URL|awk -F/ '{print $NF}'`  # get the package name from the url

# Check to see if msgpack has been built already
function check_msgpack_built()
{
    # Check for a file that's created when msgpack is configured   
    fname=$(find "$MSGPACK_BUILD_DIR" -iname config.status 2>/dev/null)
    if [ ! -f "$fname" ]; then
        return $FALSE
    fi
    
    # Check for a file that's created when msgpack is made
    fname=$(find "$MSGPACK_BUILD_DIR/src" -iname *.o 2>/dev/null)
    if [ ! -f "$fname" ]; then
        return $FALSE
    fi
    
    return $TRUE
}


# Download the package to env/downloads
function download_msgpack()
{

    msgpack_pkg_path="$DOWNLOADS_DIR/$msgpack_pkg"

    if [ -f "$msgpack_pkg_path" ]; then
        echo "- MessagePack package already exists ('$msgpack_pkg_path') -- skipping download."
        return
    fi

    echo "Downloading $msgpack_pkg to $msgpack_pkg_path"
    download_file "$MSGPACK_URL" "$msgpack_pkg_path"
}

# Unpack to the build directory specified in install.conf
function unpack_msgpack()
{
    # Create directory and unpack
    if check_directory_exists "$MSGPACK_BUILD_DIR"; then
        echo "- MessagePack is already unpacked to '$MSGPACK_BUILD_DIR' -- skipping."
        return
    fi

    echo "Unpacking msgpack to $MSGPACK_BUILD_DIR (this may take a minute...)"
    # TODO: Do we need to remove the dir if it already exists?
    create_directory_if_noexist $MSGPACK_BUILD_DIR

    # Unzip
    pushd "$MSGPACK_BUILD_DIR" > /dev/null
    tar xf "$DOWNLOADS_DIR/$msgpack_pkg" --strip 1 || { echo "- ERROR: Failed to unpack MessagePack."; exit 1; }
    popd > /dev/null
}

# Build the package under the build directory specified in in msgpack.conf
function build_msgpack()
{

    echo "- Building MessagePack under $MSGPACK_BUILD_DIR"
    pushd "$MSGPACK_BUILD_DIR" > /dev/null

    # Perform the build
    # NOTE: Need 'CFLAGS="-O0"' to fix https://github.com/msgpack/msgpack-c/issues/3
    ./configure --prefix="$MSGPACK_INSTALL_PREFIX" CFLAGS='-O0'
    
    make || { echo "- ERROR: MessagePack 'make' failed."; exit 1; }

    popd > /dev/null
}

# Install the package under the package install prefix from install.conf
function install_msgpack()
{

    echo "- Installing MessagePack under $MSGPACK_INSTALL_PREFIX"

    pushd "$MSGPACK_BUILD_DIR" > /dev/null

    make install || { echo "- ERROR: MessagePack 'make install' failed."; exit 1; }

    popd > /dev/null
}



function main()
{

    ensure_install_prefix_writable $MSGPACK_INSTALL_PREFIX
    
    if check_package_installed "$MSGPACK_INSTALL_PREFIX/lib/libmsgpack*"; then
        echo "- MessagePack is installed under prefix $MSGPACK_INSTALL_PREFIX -- skipping."
        return
    fi

    if check_msgpack_built; then
        echo "- MessagePack is already built under $MSGPACK_BUILD_DIR -- skipping."
        install_msgpack
        return
    fi

    if check_file_exists "$MSGPACK_PACKAGE_DIR/configure"; then
        echo "- MessagePack is already unpacked to $MSGPACK_BUILD_DIR -- skipping."
        build_msgpack
        install_msgpack
        return
    fi

    if check_file_exists "$DOWNLOADS_DIR/$msgpack_pkg"; then
        echo "- MessagePack package already exists under env/downloads -- skipping download."
        unpack_msgpack
        build_msgpack
        install_msgpack
        return
    fi



    # If we haven't returned by now, we have to do everything
    download_msgpack
    unpack_msgpack
    build_msgpack
    install_msgpack

}

main
